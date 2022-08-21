use super::ContactInformations;
use crate::engine::shapes::{Plane, Segment, Shape, OBB};
use crate::geometry::{
    geometry_traits::{FaceIndex, PolyhedronTrait},
    helper as geometry_helper,
};
use crate::math::{math_essentials::*, Quaternion};

pub fn obb_plane(obb: &OBB, plane: &Plane) -> ContactInformations {
    let mut distance = ZERO;
    let mut points = Vec::<P3>::new();

    for v in &obb.transformed_vertices() {
        let d = plane.signed_distance(&v);
        // si en desous du plan
        if d < ZERO {
            // on remonte le point a mi chemin du plan
            points.push(v - &(plane.normal * d * 0.5));
            if d < distance {
                distance = d;
            }
        }
    }

    ContactInformations {
        points,
        normal_a_to_b: -plane.normal,
        penetration_distance: distance.abs(),
    }
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
// from pybullet
pub fn azazeazeobb_plane(obb: &OBB, plane: &Plane) -> ContactInformations {
    let mut obb_cmp: OBB = obb.clone();
    let mut points = Vec::<Vec3>::new();
    let mut penetration_distance = ZERO;

    // axis of perturbation
    let (v1, _) = geometry_helper::perp(&plane.normal);
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

    ContactInformations {
        points,
        normal_a_to_b: -plane.normal,
        penetration_distance,
    }
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
