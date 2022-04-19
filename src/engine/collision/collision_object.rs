use crate::engine::shapes::Shape;
use crate::math::math_essentials::*;

pub struct CollisionObject {
    pub enabled: bool,
    pub id: u32,
    pub shape: Box<dyn Shape>,
}

impl CollisionObject {
    pub fn new(shape: Box<dyn Shape>) -> CollisionObject {
        CollisionObject {
            enabled: true,
            id: 0,
            shape: shape,
        }
    }

    
}
