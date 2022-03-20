use super::{Shape, ShapeType};
use crate::math::{math_essentials::*, Mat3};

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
        let mut d = p - &self.position;
        normalize(&mut d);
        self.position + d * self.radius
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
