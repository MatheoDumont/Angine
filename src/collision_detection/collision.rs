use crate::math::{P3, Vec3, Transform};
use super::shapes::Shape;

pub struct CollisionData {
    pub point: P3,
    pub normal_a_to_b: Vec3,
}

pub struct CollisionObject {
    pub enabled: bool,
    pub id: u32,
    pub shape: Box<dyn Shape>,
    pub transform: Transform,
}

impl CollisionObject {
    pub fn new(transform: Transform, shape: Box<dyn Shape>) -> CollisionObject {
        CollisionObject {
            enabled: true,
            id: 0,
            shape: shape,
            transform: transform,
        }
    }
}
