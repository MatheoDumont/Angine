use self::{
    collision::{CollisionData, CollisionObject},
    intersection_algorithms::intersection_wrapper::get_intersection_fn_by_collisiontypes,
};

use std::vec::Vec;

pub mod collision;
pub mod intersection_algorithms;
pub mod shapes;

pub struct CollisionWorld {
    collision_objects: Vec<CollisionObject>,
    detected_intersections: Vec<CollisionData>,
    id_counter: u32, // used to give a number to each CollisionObject
}

impl CollisionWorld {
    pub fn new() -> Self {
        Self {
            collision_objects: Vec::new(),
            detected_intersections: Vec::new(),
            id_counter: 0,
        }
    }

    pub fn add_collision_object(&mut self, mut o: CollisionObject) {
        o.id = self.id_counter;
        self.id_counter += 1;
        self.collision_objects.push(o);
    }

    pub fn step(&'static mut self) {
        for i in 0..self.collision_objects.len() {
            let obj_i = &self.collision_objects[i];
            let pos_i = &obj_i.transform;
            let shape_i = &obj_i.shape;

            for j in (i + 1)..self.collision_objects.len() {
                let obj_j = &self.collision_objects[j];

                if let Some(algo) = get_intersection_fn_by_collisiontypes(&obj_i, &obj_j) {
                    // if let Some(intersection) =
                    //     algo.compute(pos_i, shape_i, &obj_j.position, &obj_j.shape)
                    // {
                    //     self.detected_intersections.push(intersection);
                    // }
                }
            }
        }
    }
}
