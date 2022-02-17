pub mod obb;
pub mod plane;
pub mod sphere;
pub mod obb_plane;
pub mod obb_sphere;
pub mod plane_sphere;

pub use obb::obb_obb;
pub use sphere::sphere_sphere;
pub use plane::plane_plane;
pub use obb_plane::obb_plane;
pub use obb_sphere::obb_sphere;
pub use plane_sphere::plane_sphere;

use crate::engine::collision::CollisionObject;
use crate::math::{Real, Vec3, P3};

pub struct ContactManifold {
    pub points: Vec<P3>,
    pub normal_a_to_b: Vec3,
    pub penetration_distance: Real,
    // pub a: Box<CollisionObject>,
    // pub b: Box<CollisionObject>,
}
