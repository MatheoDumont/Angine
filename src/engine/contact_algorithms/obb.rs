use super::ContactManifold;

use crate::engine::shapes::OBB;
use crate::math::{helper, sat, sat::SAT, Real, Vec3, P3, ZERO};

/**
 * Ca marche pas, il faut recenter les points pas seulement sur les sommes mais aussi sur aretes,
 * voir les faces aussi, 2 obb peuvent s'intersecter sans qu'aucun des sommets de l'un soient dans l'autre et vice versa.
 */
pub fn obb_obb(obb1: &OBB, obb2: &OBB) -> ContactManifold {
    let vertices1 = obb1.vertices();
    let vertices2 = obb2.vertices();
    // let mut centre = obb2.transform.position();
    // let mut index_closest_vertex = helper::closest_to_p(&centre, &v1);

    // if !obb2.is_inside(&v1[index_closest_vertex]) {
    //     v2 = obb2.vertices();
    //     centre = obb1.transform.position();
    //     index_closest_vertex = helper::closest_to_p(&centre, &v2);
    //     if !obb1.is_inside(&v2[index_closest_vertex]) {
    //         panic!("Calcul du ContactManifold pour l'intersection OBB/OBB : aucun points de l'un des OBB n'est dans l'autre !! Pas d'intersection ?");
    //     }
    // }

    let r = sat::SeparatingAxisMethod::compute_3D(
        &vertices1,
        &obb1.separating_axis(),
        &vertices2,
        &obb2.separating_axis(),
    ).expect("contact_algorithms::obb_obb, OBB are intersecting, SAT should return Some(x), there is a problem");

    ContactManifold {
        points: vec![], // should not be empty
        normal_a_to_b: Vec3::up(),
        penetration_distance: r.minimum_translation,
    }
}
