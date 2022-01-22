use super::{Shape, ShapeType};
use crate::math::{Real, Vec3};

pub struct Sphere {
    pub radius: Real, // radius in ]0, +inf[ but no check
}

impl Sphere {
    pub fn new(_radius: Real) -> Self {
        Self {
            radius: _radius,
        }
    }
}

impl Shape for Sphere {
    fn inertia_matrix(&self) -> Vec3 {
        // pour l'instant
        Vec3::zeros()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Sphere
    }
}
