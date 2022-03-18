pub mod helper;
pub mod matrix;
pub mod point;
pub mod transform;
pub mod vec3;
pub mod vector;
// pub mod static_square_matrix;
pub mod quaternion;
pub mod rotation_matrix;

pub use helper::*;
pub use matrix::Mat3;
pub use point::P3;
pub use quaternion::Quaternion;
pub use rotation_matrix::RotationMatrix;
pub use transform::Rotation;
pub use transform::Transform;
pub use vec3::Vec3;
pub use vector::Vector;
pub type Real = f32;

pub const ZERO: Real = 0 as Real;
pub const ONE: Real = 1 as Real;
pub const TWO: Real = 2 as Real;
