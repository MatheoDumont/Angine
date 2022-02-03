pub mod matrix;
pub mod point;
pub mod transform;
pub mod vector;
pub mod helper;

pub use matrix::Mat3;
pub use point::P3;
pub use transform::Rotation;
pub use transform::Transform;
pub use vector::Vec3;
pub use helper::*;

pub type Real = f32;

pub const ZERO: Real = 0 as Real;
pub const ONE: Real = 1 as Real;
