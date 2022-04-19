use super::{Shape, ShapeType};
use crate::math::{math_essentials::*, Mat3};

pub struct Sphere {
    pub radius: Real, // radius in ]0, +inf[ but no check
    pub position: P3,
    pub inertia_matrix: Mat3,
    pub inv_inertia_matrix: Mat3,
}

impl Sphere {
    pub fn new(radius: Real, position: P3) -> Self {
        Self {
            radius: radius,
            position: position,
            inertia_matrix: Mat3::identity(),
            inv_inertia_matrix: Mat3::identity(),
        }
    }

    pub fn closest_point(&self, p: &P3) -> P3 {
        let mut d = p - &self.position;
        normalize(&mut d);
        self.position + d * self.radius
    }
}

impl Shape for Sphere {
    fn compute_inertia_matrix(&mut self, mass: Real) {
        // pour l'instant
        panic!("inertia matrix for Sphere not implemented");
    }
    fn inertia_matrix(&self) -> &Mat3 {
        &self.inertia_matrix
    }
    fn inverse_inertia_matrix(&self) -> &Mat3 {
        &self.inv_inertia_matrix
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Sphere
    }

    fn is_rigid_body(&self) -> bool {
        true
    }

    fn get_position(&self) -> &P3 {
        &self.position
    }
    // bidon
    fn get_orientation(&self) -> &Mat3 {
        &self.inertia_matrix()
    }
    fn set_position(&mut self, p: P3) {
        self.position = p;
    }
    fn set_orientation(&mut self, o: Mat3) {}
}
