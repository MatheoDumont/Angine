use super::ContactInformation;

use crate::engine::shapes::{Segment, OBB};
use crate::geometry::{self, geometry_traits::*, sat, sat::EdgeResult, sat::FaceResult};
use crate::math::math_essentials::*;

pub fn obb_obb(obb1: &OBB, obb2: &OBB) -> ContactInformation {
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
    ContactInformation {
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
        geometry::helper::clip(&mut vertices_to_clip, side_normal, &v1);
    }

    // maintenant on clip avec la normal de la face de référence pour relever tous les points en dessous sur la face, on laisse ceux au dessus
    geometry::helper::clip(
        &mut vertices_to_clip,
        -reference_face_normal,
        &obb1_vertices[obb1.faces_ref()[reference_face.face_index].v_i[0]],
    );

    vertices_to_clip
}

pub fn edge_contact(obb1: &OBB, obb2: &OBB, edge: &EdgeResult) -> Vec<P3> {
    let e1 = &obb1.edges_ref()[edge.edge_a_index];
    let e2 = &obb2.edges_ref()[edge.edge_b_index];
    let l1 = Segment::new(
        obb1.transformed_vertex(e1.vi1),
        obb1.transformed_vertex(e1.vi2),
    );
    let l2 = Segment::new(
        obb1.transformed_vertex(e2.vi1),
        obb1.transformed_vertex(e2.vi2),
    );

    let points = l1.closest_point_each_other(&l2);

    if helper::round_n_decimal_vector(&points[0], 6)
        == helper::round_n_decimal_vector(&points[1], 6)
    {
        vec![points[0]]
    } else {
        vec![(points[0] + points[1]) / TWO]
    }
}
