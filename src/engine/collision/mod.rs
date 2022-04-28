pub mod collision_object;

pub use collision_object::CollisionObject;

use crate::engine::contact_algorithms::ContactManifold;
use crate::engine::intersection_algorithms::intersection_wrapper::get_intersection_fn_by_collisiontypes;
use crate::math::math_essentials::*;

use std::collections::HashMap;
use std::vec::Vec;

pub struct CollisionWorld {
    pub collision_objects: HashMap<usize, CollisionObject>,
    pub contact_manifolds: Vec<ContactManifold>,
    id_counter: usize, // used to give a number to each CollisionObject
}

impl CollisionWorld {
    pub fn new() -> Self {
        Self {
            collision_objects: HashMap::new(),
            contact_manifolds: Vec::new(),
            id_counter: 0,
        }
    }

    pub fn add_collision_object(&mut self, mut o: CollisionObject) -> usize {
        let id = self.id_counter;
        self.id_counter += 1;
        o.id = id;
        self.collision_objects.insert(id, o);
        id
    }

    pub fn collision_object_ref(&self, id: usize) -> Result<&CollisionObject, &str> {
        match self.collision_objects.get(&id) {
            Some(x) => Ok(x),
            None => Err("get_collision_object_ref() = Can't find the CollisionObject associated with thid id."),
        }
    }

    pub fn collision_object_mut(&mut self, id: usize) -> Result<&mut CollisionObject, &str> {
        match self.collision_objects.get_mut(&id) {
            Some(x) => Ok(x),
            None => Err("get_collision_object_mut() = Can't find the CollisionObject associated with thid id."),
        }
    }

    pub fn update_transform_collision_object(&mut self, id: usize, transform: Transform) {
        match self.collision_objects.get_mut(&id) {
            Some(x) => {
                x.shape.set_transform(transform);
            }
            None => {
                panic!("update_collision_object(id), the pair key-value doesn't exist !, the collision object isnt");
            }
        }
    }

    pub fn step(&mut self) {
        for (i1, el1) in self.collision_objects.iter().enumerate() {
            let obj_i = el1.1;
            if !obj_i.enabled {
                continue;
            }
            let shape_i = &obj_i.shape;

            for el2 in self.collision_objects.iter().nth(i1 + 1) {
                let obj_j = el2.1;
                if !obj_j.enabled || (obj_i.is_static && obj_j.is_static) {
                    continue;
                }

                let shape_j = &obj_j.shape;
                if let Some(algo) = get_intersection_fn_by_collisiontypes(shape_i, shape_j) {
                    if let Some(contact_infos) = algo(shape_i, shape_j) {
                        let cm = ContactManifold {
                            id_collision_object_a: *el1.0,
                            id_collision_object_b: *el2.0,
                            contact_infos,
                        };
                        self.contact_manifolds.push(cm);
                    }
                }
            }
        }
    }

    pub fn clear_manifold(&mut self) {
        self.contact_manifolds.clear();
    }
}
