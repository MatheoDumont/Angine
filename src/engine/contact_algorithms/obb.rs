use super::ContactManifold;

use crate::engine::shapes::OBB;
use crate::geometry::{geometry_traits::*, sat, sat::SAT};
use crate::math::{helper, Real, Vec3, P3, ZERO};
/**
 * Ca marche pas, il faut recenter les points pas seulement sur les sommes mais aussi sur aretes,
 * voir les faces aussi, 2 obb peuvent s'intersecter sans qu'aucun des sommets de l'un soient dans l'autre et vice versa.
 */
pub fn obb_obb(obb1: &OBB, obb2: &OBB) -> ContactManifold {
    // let vertices1 = obb1.vertices();
    // let vertices2 = obb2.vertices();
    // FacesA / FacesB
    // Face of A that minimize the distance to the closest point of B
    // in fact, a variant of the SAT

    // EdgesA / EdgesB

    ContactManifold {
        points: vec![], // should not be empty
        normal_a_to_b: Vec3::up(),
        penetration_distance: ZERO,
    }
}
