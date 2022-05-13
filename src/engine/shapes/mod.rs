pub mod obb;
pub mod plane;
pub mod segment;
pub mod sphere;

use crate::math::{math_essentials::*, Mat3};
use downcast_rs::DowncastSync;

pub use obb::OBB;
pub use plane::Plane;
pub use segment::Segment;
pub use sphere::Sphere;

#[derive(Copy, Clone, Debug)]
pub enum ShapeType {
    Sphere = 0,
    OBB = 1,
    Plane = 2,
    Segment = 3,
}

pub trait Shape: DowncastSync {
    fn shape_type(&self) -> ShapeType;
    fn is_rigid_body(&self) -> bool;
    fn compute_inertia_matrix(&self, mass: Real) -> Mat3;

    fn get_position(&self) -> &P3;
    fn get_orientation(&self) -> &Mat3;
    fn get_transform(&self) -> Transform;
    fn set_position(&mut self, p: P3);
    fn set_orientation(&mut self, o: Mat3);
    fn set_transform(&mut self, t: Transform);

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
