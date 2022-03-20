pub mod helper;
pub mod matrix;
pub mod quaternion;
pub mod rotation_matrix;
pub mod transform;
pub mod vector;
// pub mod static_square_matrix;

pub use helper::*;
pub use matrix::Mat3;
pub use quaternion::Quaternion;
pub use rotation_matrix::RotationMatrix;
pub use transform::Rotation;
pub use transform::Transform;
pub use vector::Directions;
pub use vector::Vector3;

pub type Real = f32;
pub type Vec3 = vector::Vector3;
pub type P3 = vector::Vector3;

pub const ZERO: Real = 0 as Real;
pub const ONE: Real = 1 as Real;
pub const TWO: Real = 2 as Real;

pub mod math_essentials {

    pub use super::{helper, vector::*, Real, Rotation, Transform, Vec3, ONE, P3, TWO, ZERO};
}
