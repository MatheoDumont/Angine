use super::ContactManifold;

use crate::engine::shapes::OBB;
use crate::geometry::{geometry_traits::*, sat, sat::Edge, sat::Face, sat::SAT};
use crate::math::math_essentials::*;
/**
 * Ca marche pas, il faut recenter les points pas seulement sur les sommes mais aussi sur aretes,
 * voir les faces aussi, 2 obb peuvent s'intersecter sans qu'aucun des sommets de l'un soient dans l'autre et vice versa.
 */
pub fn obb_obb(obb1: &OBB, obb2: &OBB) -> ContactManifold {
    let sat_result = sat::sat_3D(obb1, obb2).unwrap();

    if sat_result.face_A.distance < sat_result.edge.distance
        && sat_result.face_B.distance < sat_result.edge.distance
    {
        // Face Contact
    } else {
        // Edge Contact
    }
    ContactManifold {
        points: vec![], // should not be empty
        normal_a_to_b: Directions::up(),
        penetration_distance: ZERO,
    }
}

pub fn face_contact(obb1: &OBB, obb2: &OBB, reference_face: &Face) -> Vec<P3> {
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
            // on clip
            let clipped_point = face_to_vertex2clip + perpendicular(&clipping_normal, &vertex2clip);
            vertices_to_clip[i] = clipped_point;
        }
    }
}
