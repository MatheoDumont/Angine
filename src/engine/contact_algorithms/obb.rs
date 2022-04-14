use super::ContactManifold;

use crate::engine::shapes::{Line, OBB};
use crate::geometry::{geometry_traits::*, sat, sat::EdgeResult, sat::FaceResult};
use crate::math::math_essentials::*;

pub fn obb_obb(obb1: &OBB, obb2: &OBB) -> ContactManifold {
    let sat_result = sat::sat_3D(obb1, obb2).unwrap();

    let normal: Vec3;
    let distance: Real;
    let points: Vec<P3>;
    if sat_result.face_A.distance <= sat_result.edge.distance
        && sat_result.face_B.distance <= sat_result.edge.distance
    {
        // Face Contact
        normal = sat_result.face_A.axis;
        distance = sat_result.face_A.distance;
        points = face_contact(&obb1, &obb2, &sat_result.face_A);
    } else {
        // Edge Contact
        normal = sat_result.edge.axis;
        distance = sat_result.edge.distance;
        points = edge_contact(&obb1, &obb2, &sat_result.edge);
    }
    ContactManifold {
        points,
        normal_a_to_b: normal,
        penetration_distance: distance,
    }
}

pub fn face_contact(obb1: &OBB, obb2: &OBB, reference_face: &FaceResult) -> Vec<P3> {
    // face de obb1
    let reference_face_normal = reference_face.axis;

    // on récupère la face adjacente: plus proche face sur obb2 de la face de référence
    let mut adjacent_face_index = 0;

    let mut dot_result = dot(
        &reference_face_normal,
        &obb2.face_normal(adjacent_face_index),
    );
    let obb1_vertices = obb1.transformed_vertices();
    let obb2_vertices = obb2.transformed_vertices();

    for i in 1..obb2.sizes().faces {
        let i_face_normal = obb2.face_normal(i);
        let x = dot(&reference_face_normal, &i_face_normal);
        if x < dot_result {
            adjacent_face_index = i;
            dot_result = x;
        }
    }
    // on récupère les vertex de la face adjacente
    let mut vertices_to_clip = Vec::<P3>::new();
    vertices_to_clip.reserve(obb2.faces_ref()[adjacent_face_index].v_i.len());
    for i in &obb2.faces_ref()[adjacent_face_index].v_i {
        vertices_to_clip.push(obb2_vertices[*i]);
    }

    // on clip pour chaque côté de la face de référence les vertices_to_clip
    let obb1_ref_face_vertices_index = &obb1.faces_ref()[reference_face.face_index];
    let n = obb1_ref_face_vertices_index.v_i.len();
    for i in 0..n {
        // on calcul la normal
        let v1 = &obb1_vertices[obb1_ref_face_vertices_index.v_i[i]];
        let v2 = &obb1_vertices[obb1_ref_face_vertices_index.v_i[(i + 1) % n]];

        let mut side_normal = cross(&reference_face_normal, &(v1 - v2));
        normalize(&mut side_normal);
        if dot(&side_normal, &v1) < ZERO {
            side_normal = -side_normal;
        }

        // on clip
        clip(&mut vertices_to_clip, side_normal, &v1);
    }

    // maintenant on clip avec la normal de la face de référence pour relever tous les points en dessous sur la face, on laisse ceux au dessus
    clip(
        &mut vertices_to_clip,
        -reference_face_normal,
        &obb1_vertices[obb1.faces_ref()[reference_face.face_index].v_i[0]],
    );

    vertices_to_clip
}

pub fn edge_contact(obb1: &OBB, obb2: &OBB, edge: &EdgeResult) -> Vec<P3> {
    let e1 = &obb1.edges_ref()[edge.edge_a_index];
    let e2 = &obb2.edges_ref()[edge.edge_b_index];
    let l1 = Line::new(
        obb1.transformed_vertex(e1.vi1),
        obb1.transformed_vertex(e1.vi2),
    );
    let l2 = Line::new(
        obb1.transformed_vertex(e2.vi1),
        obb1.transformed_vertex(e2.vi2),
    );

    let points = l1.closest_point_each_other(&l2);

    if helper::same_points(&points[0], &points[1]) {
        vec![points[0]]
    } else {
        vec![(points[0] + points[1]) / TWO]
    }
}

/**
 * pour tous les points de adjacent_face_index ou dot(&p, side_normal) > 0, ramener pour dot(&p, side_normal) == 0
 * ou p est un des points composant la face adjacent_face_index
 */
fn clip(vertices_to_clip: &mut Vec<P3>, clipping_normal: Vec3, vertex_on_face: &Vec3) {
    // for adj_face_vertex_index in &obb2.faces_ref()[adjacent_face_index].v_i {
    for i in 0..vertices_to_clip.len() {
        let vertex2clip = &vertices_to_clip[i];
        let face_to_vertex2clip = vertex2clip - vertex_on_face;

        if dot(&clipping_normal, &vertex2clip) > ZERO {
            vertices_to_clip[i] =
                vertex2clip - &projection(&clipping_normal, &(vertex2clip - vertex_on_face));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    #[test]
    fn test_clip() {
        {
            let mut v = vec![
                P3::new(1.0, 2.0, 3.0),
                P3::new(-1.0, -2.0, -3.0),
                P3::new(1.0, 0.0, 3.0),
            ];
            clip(&mut v, Directions::up(), &P3::origin());

            assert_eq!(v[0].x(), ONE);
            assert_eq!(v[0].y(), ZERO);
            assert_eq!(v[0].z(), 3.0);

            assert_eq!(v[1].x(), -ONE);
            assert_eq!(v[1].y(), -2.0);
            assert_eq!(v[1].z(), -3.0);

            assert_eq!(v[2].x(), ONE);
            assert_eq!(v[2].y(), ZERO);
            assert_eq!(v[2].z(), 3.0);
        }

        {
            let mut v = vec![
                P3::new(1.0, 2.0, 3.0),
                P3::new(-1.0, -2.0, -3.0),
                P3::new(1.0, 0.0, 3.0),
            ];
            clip(&mut v, Directions::right(), &P3::origin());

            assert_eq!(v[0].x(), ZERO);
            assert_eq!(v[0].y(), 2.0);
            assert_eq!(v[0].z(), 3.0);

            assert_eq!(v[1].x(), -ONE);
            assert_eq!(v[1].y(), -2.0);
            assert_eq!(v[1].z(), -3.0);

            assert_eq!(v[2].x(), ZERO);
            assert_eq!(v[2].y(), ZERO);
            assert_eq!(v[2].z(), 3.0);
        }

        {
            let mut v = vec![
                P3::new(1.0, 2.0, 3.0),
                P3::new(-1.0, -2.0, -3.0),
                P3::new(1.0, 0.0, 3.0),
            ];
            let normal = Rotation::Z(helper::angle_2_rad(45.0)) * Directions::right();

            clip(&mut v, normal, &P3::origin());

            assert_approx_eq!(dot(&v[0], &normal), ZERO);
            assert_approx_eq!(v[1].x(), -ONE);
            assert_approx_eq!(v[1].y(), -2.0);
            assert_approx_eq!(v[1].z(), -3.0);
            assert_approx_eq!(dot(&v[2], &normal), ZERO);
        }
    }
}
