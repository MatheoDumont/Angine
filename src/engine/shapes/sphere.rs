use super::{Shape, ShapeType};
use crate::math::{Mat3, Real, Transform, Vec3, P3};

pub struct Sphere {
    pub radius: Real, // radius in ]0, +inf[ but no check
    pub position: P3,
}

impl Sphere {
    pub fn new(radius: Real, position: P3) -> Self {
        Self {
            radius: radius,
            position: position,
        }
    }

    pub fn closest_point(&self, p: &P3) -> P3 {
        &self.position + (p - &self.position).normalized() * self.radius
    }
}

impl Shape for Sphere {
    fn inertia_matrix(&self, mass: Real) -> Mat3 {
        // pour l'instant
        panic!("inertia matrix for Sphere not implemented");
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Sphere
    }
}
