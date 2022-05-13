use super::ContactInformation;
use crate::engine::shapes::{Plane, Shape, Sphere};

pub fn plane_sphere(plane: &Plane, sphere: &Sphere) -> ContactInformation {
    let sphere_pos = sphere.get_position();
    let d = plane.signed_distance(&sphere_pos);
    ContactInformation {
        points: vec![sphere_pos - &(plane.normal * d)],
        normal_a_to_b: -plane.normal,
        penetration_distance: d,
    }
}
