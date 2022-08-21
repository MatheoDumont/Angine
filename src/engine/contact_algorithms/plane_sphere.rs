use super::ContactInformations;
use crate::engine::shapes::{Plane, Shape, Sphere};

pub fn plane_sphere(plane: &Plane, sphere: &Sphere) -> ContactInformations {
    let sphere_pos = sphere.get_position();
    let d = plane.signed_distance(&sphere_pos);
    ContactInformations {
        points: vec![sphere_pos - &(plane.normal * d)],
        normal_a_to_b: plane.normal,
        penetration_distance: d,
    }
}
