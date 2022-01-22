use crate::math::{P3, Vec3, Real};
use super::shapes::Shape;

pub struct CollisionData {
    pub point: P3,
    pub normal_a_to_b: Vec3,
}

pub struct CollisionObject {
    pub enabled: bool,
    pub id: u32,
    pub shape: Box<dyn Shape>,
    pub position: P3,
}

impl CollisionObject {
    pub fn new(position: P3, shape: Box<dyn Shape>) -> CollisionObject {
        CollisionObject {
            enabled: true,
            id: 0,
            shape: shape,
            position: position,
        }
    }
}
