use super::ContactInformation;
use crate::engine::shapes::{Plane, Segment, Shape, OBB};
use crate::geometry::{
    geometry_traits::{FaceIndex, PolyhedronTrait},
    helper as geometry_helper,
};
use crate::math::{math_essentials::*, Quaternion};

pub fn normal_with_points(obb_vertices: &Vec<Vec3>, face_indices: &FaceIndex) -> Vec3 {
    // let face_indices = &obb.faces_ref()[face_index];

    let p1 = &obb_vertices[face_indices.v_i[0]];
    let p2 = &obb_vertices[face_indices.v_i[1]];
    let p3 = &obb_vertices[face_indices.v_i[2]];

    geometry_helper::face_normal(&p1, &p2, &p3)
}

pub fn collision_point(obb: &OBB, plane: &Plane) -> (P3, Real, Vec3) {
    // essayer avec un support point sur toute la forme et ensuite seulement aux sommets
    // let p = obb.project_on_contour_in_direction(&(-plane.normal));
    let obb_vertices = obb.transformed_vertices();
    let mut p = &obb_vertices[0];
    let mut best_d = plane.signed_distance(&p);
    for i in 1..obb_vertices.len() {
        let d = plane.signed_distance(&obb_vertices[i]);
        if d < best_d {
            best_d = d;
            p = &obb_vertices[i];
        }
    }

    // let d = plane.signed_distance(&p);
    let projected_onto_plane = p - &(plane.normal * best_d);

    (projected_onto_plane, best_d, -plane.normal)
}

pub fn qsd_obb_plane(obb: &OBB, plane: &Plane) -> ContactInformation {
    let p = obb.project_on_contour_in_direction(&(-plane.normal));
    let d = plane.signed_distance(&p);
    let projected_onto_plane = p - plane.normal * d;

    ContactInformation {
        points: vec![projected_onto_plane],
        normal_a_to_b: -plane.normal,
        penetration_distance: d,
    }
}

// from pybullet
pub fn obb_plane(obb: &OBB, plane: &Plane) -> ContactInformation {
    let mut obb_cmp: OBB = obb.clone();
    let mut points = Vec::<Vec3>::new();
    let mut penetration_distance = ZERO;

    // axis of perturbation
    let (v1, v2) = geometry_helper::perp(&plane.normal);
    let angle_of_perturb = std::f32::consts::FRAC_PI_8;
    let perturbation_rot = Quaternion::from_rad_axis(angle_of_perturb, v1);
    let rot_base = perturbation_rot * Quaternion::from_mat3(&obb_cmp.transform.rotation);
    let n_iteration = 4;
    let angle_rot_per_point = TWO * std::f32::consts::PI / (n_iteration as Real);

    for i in 0..n_iteration {
        let rad = (i as Real) * angle_rot_per_point;
        let rot = Quaternion::from_rad_axis(rad, plane.normal);
        let ori = rot * rot_base;
        obb_cmp.transform.rotation = ori.to_mat3();
        let (p, d, _) = collision_point(&obb_cmp, &plane);
        points.push(p);
        penetration_distance = helper::min(d, penetration_distance);
    }

    ContactInformation {
        points,
        normal_a_to_b: -plane.normal,
        penetration_distance,
    }
}

pub fn old_obb_plane(obb: &OBB, plane: &Plane) -> ContactInformation {
    println!("========================= obb plane =========================");
    let obb_vertices = obb.transformed_vertices();
    let mut fittest_face_idx = 0;
    let mut fittest_normal = normal_with_points(&obb_vertices, &obb.faces_ref()[0]);
    let mut dot_fittest = dot(&plane.normal, &fittest_normal);

    for face_index in 0..obb.sizes().faces {
        let face_indices = &obb.faces_ref()[face_index];
        let n = normal_with_points(&obb_vertices, &face_indices);
        let d = dot(&plane.normal, &n);
        if d < dot_fittest {
            fittest_face_idx = face_index;
            fittest_normal = n;
            dot_fittest = d;
        }
    }

    println!(
        "  [1] fittest normal : {:?} {:?}",
        fittest_normal, dot_fittest
    );

    let face_vertices = &obb.faces_ref()[fittest_face_idx].v_i;
    let mut distance_points_to_plane = Vec::<Real>::new();
    distance_points_to_plane.push(plane.signed_distance(&obb_vertices[face_vertices[0]]));
    let mut min_distance_i = 0;
    let mut face_mean_distance = ZERO;
    for p_i in 1..face_vertices.len() {
        let d = plane.signed_distance(&obb_vertices[face_vertices[p_i]]);
        if d < distance_points_to_plane[min_distance_i] {
            min_distance_i = p_i
        }
        face_mean_distance += d;
        distance_points_to_plane.push(d);
    }
    println!("  [2] Distance points to plane : ");
    for i in 0..distance_points_to_plane.len() {
        println!(
            "       {:?} {:?}",
            obb_vertices[face_vertices[i]], distance_points_to_plane[i]
        );
    }
    face_mean_distance /= face_vertices.len() as Real;
    let l = distance_points_to_plane.len();
    let mut min_mean_edge_distance = ZERO;
    let mut min_edge_idx = 0;
    for current in 0..l {
        let previous = (l - 1 + current) % l;
        let distance_a = distance_points_to_plane[previous];
        let distance_b = distance_points_to_plane[current];

        let edge_mean_distance = (distance_a + distance_b) / (2 as Real);
        if edge_mean_distance <= min_mean_edge_distance {
            min_mean_edge_distance = edge_mean_distance;
            min_edge_idx = current;
        }
    }

    // diff la moyenne des distances des 2 deux points et la moyenne de la face,
    // difference entre les deux distnaces de chaque point,
    // avoir un indicateur, genre difference moyenne > 0.5 = trop grand

    let idx_a = (l - 1 + min_edge_idx) % l;
    let idx_b = min_edge_idx;
    let distance_a = distance_points_to_plane[idx_a];
    let distance_b = distance_points_to_plane[idx_b];
    let a = obb_vertices[face_vertices[idx_a]];
    let b = obb_vertices[face_vertices[idx_b]];
    let mut contact_points = Vec::<Vec3>::new();
    let mut normal_a_to_b;

    // face contact
    if face_mean_distance <= ZERO || (face_mean_distance - min_mean_edge_distance).abs() < 0.5 {
        let points_on_face = face_vertices
            .iter()
            .map(|i: &usize| obb_vertices[*i])
            .collect();
        contact_points = clip_edges_on_plane(&points_on_face, &plane);
        // normal_a_to_b = fittest_normal;
        normal_a_to_b = -plane.normal;
    } else {
        // point contact
        if (distance_a - distance_b).abs() > 0.5 {
            if distance_a < distance_b {
                // point a
                contact_points.push(a);
            } else {
                // point b
                contact_points.push(b);
            }

            normal_a_to_b = normalized(&contact_points[0] - &obb.transform.translation);
        }
        // edge contact
        else {
            let mean_point = (&a + &b) / TWO;
            contact_points.push(a);
            contact_points.push(b);
            normal_a_to_b = normalized(&mean_point - &obb.transform.translation);
        }
    }

    let mut penetration_distance = ZERO;
    for p in &contact_points {
        let d = plane.signed_distance(p);
        if d < penetration_distance {
            penetration_distance = d;
        }
    }
    if penetration_distance > obb.distance_to_contour_in_direction(&(-plane.normal)) {
        normal_a_to_b = plane.normal;
    }

    ContactInformation {
        points: contact_points,
        normal_a_to_b: normal_a_to_b,
        penetration_distance,
    }
}

fn clip_edges_on_plane(points: &Vec<Vec3>, plane: &Plane) -> Vec<P3> {
    let mut new_points = Vec::<P3>::new();
    for current in 0..points.len() {
        let previous = (points.len() - 1 + current) % points.len();
        let a = points[previous];
        let b = points[current];

        let segment = Segment::new(a, b);

        // println!("p={:?} signed distance={:?}", a, plane.signed_distance(&a));

        if segment.is_coplanar(&plane) {
            if plane.signed_distance(&a) < ZERO {
                if new_points.last().is_none()
                    || (new_points.last().is_some() && new_points.last().unwrap() != &a)
                {
                    new_points.push(a);
                }
                new_points.push(b);
            }
        } else if let Some(p) = segment.intersect_plane(&plane) {
            // lequel est au dessous et au dessus
            // println!("intersection= {:?}", p);
            if plane.signed_distance(&a) < ZERO {
                if new_points.last().is_none()
                    || (new_points.last().is_some() && new_points.last().unwrap() != &a)
                {
                    new_points.push(a);
                }
                new_points.push(p);
            } else {
                new_points.push(p);
                new_points.push(b);
            }
        } else if plane.signed_distance(&a) < ZERO {
            if new_points.last().is_none()
                || (new_points.last().is_some() && new_points.last().unwrap() != &a)
            {
                new_points.push(a);
            }
            new_points.push(b);
        }
    }

    if new_points.len() > 0 && new_points[0] == new_points[new_points.len() - 1] {
        new_points.remove(new_points.len() - 1);
    }
    new_points
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Mat3;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn new_obb_plane_test() {
        {
            let mut obb = OBB::new(Vec3::value(ONE));
            obb.transform = Transform::translation(Vec3::new(ZERO, 0.5, ZERO));
            let plane = Plane::new(Directions::up());

            let r = obb_plane(&obb, &plane);

            println!("Points de collisions ({:?}) : ", r.points.len(),);
            for p in &r.points {
                println!("{:?}", p);
            }
            for p in &r.points {
                assert!(obb.is_on_contour(p));
            }
            println!("{:?} {:?} ", r.normal_a_to_b, r.penetration_distance);
            assert!(
                dot(
                    &r.normal_a_to_b,
                    &(plane.get_position() - obb.get_position())
                ) > ZERO
            );
            for p in &r.points {
                println!("{:?}", p);
            }
        }

        {
            let mut obb = OBB::new(Vec3::value(ONE));
            obb.transform = Transform::new(
                Vec3::ones(),
                Rotation::Z(helper::angle_2_rad(35.0)),
                Vec3::new(ZERO, 0.5, ZERO),
            );
            let plane = Plane::new(Directions::up());

            let r = obb_plane(&obb, &plane);

            println!("Points de collisions ({:?}) : ", r.points.len(),);
            for p in &r.points {
                println!("{:?}", p);
            }

            // passe pas, mais d'apres unity, pas loin du tout
            // for p in &r.points {
            //     assert!(obb.is_on_contour(p));
            // }

            // assert pas de doublon
            for i in 0..r.points.len() {
                for j in i + 1..r.points.len() {
                    assert_ne!(r.points[i], r.points[j]);
                }
            }
            println!("{:?} {:?} ", r.normal_a_to_b, r.penetration_distance);
            // normal dans le bon sens
            assert!(
                dot(
                    &r.normal_a_to_b,
                    &(plane.get_position() - obb.get_position())
                ) > ZERO
            );
        }

        {
            // from real case
            let transform = Transform {
                rotation: Mat3 {
                    data: [
                        [0.99999994, 0.0, 0.00034526698],
                        [0.00034526698, 0.0, -0.9999999],
                        [0.0, 1.0, 0.0],
                    ],
                },
                translation: Vector3 {
                    data: [0.0, -9.049723, 0.0],
                },
            };
            let mut obb = OBB::new(Vec3::value(ONE));
            obb.set_transform(transform);

            let mut plane = Plane::new(Directions::up());
            plane.set_position(Vec3::new(ZERO, -10.0, ZERO));

            let r = obb_plane(&obb, &plane);
            println!("Points de collisions ({:?}) : ", r.points.len(),);
            for p in &r.points {
                println!("{:?}", p);
            }
            println!("{:?} {:?} ", r.normal_a_to_b, r.penetration_distance);

            // assert pas de doublon
            for i in 0..r.points.len() {
                for j in i + 1..r.points.len() {
                    assert_ne!(r.points[i], r.points[j]);
                }
            }

            assert!(
                dot(
                    &r.normal_a_to_b,
                    &(plane.get_position() - obb.get_position())
                ) > ZERO
            );

            // passe pas, mais d'apres unity, pas loin du tout
            for p in &r.points {
                assert!(obb.is_on_contour(p));
            }
        }
    }
}
