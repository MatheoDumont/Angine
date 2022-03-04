pub mod geometry_traits;
pub mod helper;
pub mod matrix;
pub mod point;
pub mod sat;
pub mod transform;
pub mod vector;

pub use helper::*;
pub use matrix::Mat3;
pub use point::P3;
pub use transform::Rotation;
pub use transform::Transform;
pub use vector::Vec3;

pub type Real = f32;

pub const ZERO: Real = 0 as Real;
pub const ONE: Real = 1 as Real;
pub const TWO: Real = 2 as Real;
