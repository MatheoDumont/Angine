pub mod collision_object;

pub use collision_object::CollisionObject;

use crate::engine::contact_algorithms::ContactManifold;
use crate::engine::intersection_algorithms::intersection_wrapper::get_intersection_fn_by_collisiontypes;
use crate::math::math_essentials::*;

use std::collections::HashMap;
use std::rc::Rc;
use std::vec::Vec;

pub struct CollisionWorld {
    pub collision_objects: HashMap<usize, Rc<CollisionObject>>,
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

    pub fn add_collision_object(&mut self, mut o: CollisionObject) {
        o.id = self.id_counter;
        self.id_counter += 1;
        let rc = Rc::new(o);
        self.collision_objects.insert(Rc::as_ref(&rc).id, rc);
    }

    pub fn get_collision_object(&self, id: usize) -> Result<&Rc<CollisionObject>, &str> {
        match self.collision_objects.get(&id) {
            Some(x) => Ok(x),
            None => Err("Can't find the CollisionObject associated with thid id."),
        }
    }

    pub fn update_transform_collision_object(&mut self, id: usize, transform: Transform) {
        match self.collision_objects.get_mut(&id) {
            Some(x) => {
                if let Some(obj) = Rc::get_mut(x) {
                    obj.shape.set_orientation(transform.rotation);
                    obj.shape.set_position(transform.translation);
                } else {
                    panic!("RC collision object cannot be mutated");
                }
            }
            None => {
                panic!("update_collision_object(id), the pair key-value doesn't exist !, the collision object isnt");
            }
        }
    }

    pub fn step(&mut self) {
        for (i1, el1) in self.collision_objects.iter().enumerate() {
            let obj_i = Rc::as_ref(el1.1);
            if !obj_i.enabled {
                continue;
            }
            let shape_i = &obj_i.shape;

            for el2 in self.collision_objects.iter().nth(i1 + 1) {
                let obj_j = Rc::as_ref(el2.1);
                if !obj_j.enabled {
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
}
