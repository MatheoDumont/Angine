pub mod convex_shape;
pub mod sphere;

pub use convex_shape::ConvexMesh;
pub use convex_shape::Cube;
pub use convex_shape::Rectangle;
pub use sphere::Sphere;

use crate::math::{Real, Vec3, P3};
use downcast_rs::DowncastSync;

#[derive(Copy, Clone)]
pub enum ShapeType {
    Sphere,
    Cube,
    Rectangle,
    ConvexMesh,
}

pub trait Shape: DowncastSync {
    fn inertia_matrix(&self) -> Vec3;
    fn shape_type(&self) -> ShapeType;
}

impl_downcast!(sync Shape);
