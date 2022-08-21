use crate::engine::shapes::Shape;
use crate::math::math_essentials::*;

pub struct CollisionObject {
    pub enabled: bool,
    pub id: usize,
    pub shape: Box<dyn Shape>,
    pub rigidbody_id: Option<usize>,
    pub is_static: bool,
}

impl CollisionObject {
    pub fn new(shape: Box<dyn Shape>) -> CollisionObject {
        CollisionObject {
            enabled: true,
            id: 0,
            shape: shape,
            rigidbody_id: None,
            is_static: false,
        }
    }
}
