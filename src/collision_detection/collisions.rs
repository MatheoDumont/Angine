use crate::math::{point::Point, vector::Vector, Real};

use downcast_rs::DowncastSync;

#[derive(Copy, Clone)]
pub enum ShapeType {
    Sphere,
}

pub trait Shape: DowncastSync {
    fn inertia_matrix(&self) -> Vector;
    fn collision_type(&self) -> ShapeType;
}

impl_downcast!(sync Shape);

pub struct Sphere {
    collision_type: ShapeType,
    pub radius: Real, // radius in ]0, +inf[ but no check
}

impl Shape for Sphere {
    fn inertia_matrix(&self) -> Vector {
        // pour l'instant
        Vector::zeros()
    }

    fn collision_type(&self) -> ShapeType {
        return self.collision_type;
    }
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
