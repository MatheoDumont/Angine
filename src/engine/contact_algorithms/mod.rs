pub mod obb;
pub mod obb_plane;
pub mod obb_sphere;
pub mod plane;
pub mod plane_sphere;
pub mod sphere;

pub use obb::obb_obb;
pub use obb_plane::obb_plane;
pub use obb_sphere::obb_sphere;
pub use plane::plane_plane;
pub use plane_sphere::plane_sphere;
pub use sphere::sphere_sphere;

use crate::engine::collision::CollisionObject;
use crate::math::{Real, Vec3, P3};

use std::rc::Weak;
pub struct ContactManifold {
    pub collision_object_a: Weak<CollisionObject>,
    pub collision_object_b: Weak<CollisionObject>,
    pub contact_infos: ContactInformation,
}

pub struct ContactInformation {
    pub points: Vec<P3>,
    pub normal_a_to_b: Vec3,
    pub penetration_distance: Real,
}
