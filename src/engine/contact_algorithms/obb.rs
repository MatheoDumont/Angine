use super::ContactManifold;

use crate::engine::shapes::OBB;
use crate::math::{helper, Real, Vec3, P3, ZERO};

pub fn obb_obb(obb1: &OBB, obb2: &OBB) -> ContactManifold {
    let mut v = obb1.vertices();
    let mut centre = obb2.transform.position();
    let mut index_closest_vertex = helper::closest_to_p(&centre, &v);

    if !obb2.is_inside(&v[index_closest_vertex]) {
        v = obb2.vertices();
        centre = obb1.transform.position();
        index_closest_vertex = helper::closest_to_p(&centre, &v);
        if !obb1.is_inside(&v[index_closest_vertex]) {
            panic!("Calcul du ContactManifold pour l'intersection OBB/OBB : aucun points de l'un des OBB n'est dans l'autre !! ");
        }
    }
    
    ContactManifold {
        points: vec![v[index_closest_vertex]],
        normal_a_to_b: Vec3::up(),
        penetration_distance: ZERO,
    }
}
