pub mod sphere;
pub mod plane;
pub mod obb;

pub use sphere::Sphere;
pub use plane::Plane;
pub use obb::OBB;

use crate::math::{Mat3, Real, Transform, Vec3, P3};
use downcast_rs::DowncastSync;

#[derive(Copy, Clone)]
pub enum ShapeType {
    Sphere,
    OBB,
    Plane,
}

pub trait Shape: DowncastSync {
    fn inertia_matrix(&self) -> Vec3;
    fn shape_type(&self) -> ShapeType;
    // fn transform_ref(&self) -> &Transform;
    // fn set_transform(&self, t: Transform);
}

impl_downcast!(sync Shape);
