pub mod obb;
pub mod plane;
pub mod sphere;

pub use obb::OBB;
pub use plane::Plane;
pub use sphere::Sphere;

use crate::math::{Mat3, Real, Transform, Vec3, P3};
use downcast_rs::DowncastSync;

#[derive(Copy, Clone)]
pub enum ShapeType {
    Sphere = 0,
    OBB = 1,
    Plane = 2,
}

pub trait Shape: DowncastSync {
    fn inertia_matrix(&self, mass: Real) -> Mat3;
    fn shape_type(&self) -> ShapeType;
    // fn transform_ref(&self) -> &Transform;
    // fn set_transform(&self, t: Transform);
}

impl_downcast!(sync Shape);

#[cfg(test)]
mod tests {
    use super::*;

    fn presque_bidon() {
        assert!(ShapeType::Sphere as usize == 0usize);
        assert!(ShapeType::OBB as usize == 1usize);
        assert!(ShapeType::Plane as usize == 2usize);
    }
}
