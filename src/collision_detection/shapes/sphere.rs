use super::{Shape, ShapeType};
use crate::math::{Real, Vector};

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
    fn inertia_matrix(&self) -> Vector {
        // pour l'instant
        Vector::zeros()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Sphere
    }
}
