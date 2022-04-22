pub mod collision_object;

pub use collision_object::CollisionObject;

use crate::engine::contact_algorithms::ContactManifold;
use crate::engine::intersection_algorithms::intersection_wrapper::get_intersection_fn_by_collisiontypes;
use crate::math::{Vec3, P3};

use std::collections::HashMap;
use std::rc::{Rc, Weak};
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
        self.collision_objects.insert(rc.as_ref().id, rc);
    }

    pub fn step(&mut self) {
        for (i1, el1) in self.collision_objects.iter().enumerate() {
            let obj_i = el1.1.as_ref();
            let shape_i = &obj_i.shape;

            for el2 in self.collision_objects.iter().nth(i1 + 1) {
                let obj_j = el2.1.as_ref();
                let shape_j = &obj_j.shape;

                if let Some(algo) = get_intersection_fn_by_collisiontypes(shape_i, shape_j) {
                    if let Some(contact_infos) = algo(shape_i, shape_j) {
                        let cm = ContactManifold {
                            collision_object_a: Rc::downgrade(el1.1),
                            collision_object_b: Rc::downgrade(el2.1),
                            contact_infos,
                        };
                        self.contact_manifolds.push(cm);
                    }
                }
            }
        }
    }
}
