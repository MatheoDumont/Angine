use crate::math::{point::Point, vector::Vector, Real};
use super::shapes::Shape;

pub struct CollisionData {
    pub point: Point,
    pub normal_a_to_b: Vector,
}

pub struct CollisionObject {
    pub enabled: bool,
    pub id: u32,
    pub shape: Box<dyn Shape>,
    pub position: Point,
}

impl CollisionObject {
    pub fn new(position: Point, shape: Box<dyn Shape>) -> CollisionObject {
        CollisionObject {
            enabled: true,
            id: 0,
            shape: shape,
            position: position,
        }
    }
}
