pub mod collision_object;

pub use collision_object::CollisionObject;

use crate::engine::intersection_algorithms::intersection_wrapper::get_intersection_fn_by_collisiontypes;
use crate::math::{Vec3, P3};

use std::vec::Vec;

pub struct ContactManifold {
    pub point: P3,
    pub normal_a_to_b: Vec3,
    pub o1: Box<CollisionObject>,
    pub o2: Box<CollisionObject>,
}

pub struct CollisionWorld {
    collision_objects: Vec<Box<CollisionObject>>,
    contact_manifolds: Vec<ContactManifold>,
    id_counter: u32, // used to give a number to each CollisionObject
}

impl CollisionWorld {
    pub fn new() -> Self {
        Self {
            collision_objects: Vec::new(),
            contact_manifolds: Vec::new(),
            id_counter: 0,
        }
    }

    pub fn add_collision_object(&mut self, mut o: Box<CollisionObject>) {
        o.as_mut().id = self.id_counter;
        self.id_counter += 1;
        self.collision_objects.push(o);
    }

    pub fn step(&'static mut self) {
        for i in 0..self.collision_objects.len() {
            let obj_i = &self.collision_objects[i];
            let shape_i = &obj_i.shape;

            for j in (i + 1)..self.collision_objects.len() {
                let obj_j = &self.collision_objects[j];
                let shape_j = &obj_j.shape;

                if let Some(algo) = get_intersection_fn_by_collisiontypes(shape_i, shape_j) {
                    if algo(shape_i, shape_j) {
                        // self.contact_manifolds.push(value: T)
                    }
                }
            }
        }
    }
}
