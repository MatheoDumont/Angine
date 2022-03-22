use super::ContactManifold;

use crate::engine::shapes::OBB;
use crate::geometry::{geometry_traits::*, sat, sat::SAT};
use crate::math::{helper, Directions, Real, Vec3, P3, ZERO};
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


