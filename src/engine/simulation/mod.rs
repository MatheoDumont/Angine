pub mod collision_solver;
pub mod rigid_body;

// pub use collision_solver;
pub use rigid_body::RigidBody;

use crate::engine::collision::{CollisionObject, CollisionWorld};
use crate::engine::contact_algorithms::{ContactInformation, ContactManifold};
use crate::math::math_essentials::*;

pub struct SimulationWorld {
    bodies: Vec<RigidBody>,
    time_step: Real,
    pub collision_world: CollisionWorld,
}
use std::rc::Rc;
impl SimulationWorld {
    pub fn new(time_step: Real) -> SimulationWorld {
        SimulationWorld {
            bodies: Vec::new(),
            time_step,
            collision_world: CollisionWorld::new(),
        }
    }

    pub fn add_rigidbody(&mut self, mut rb: RigidBody, co: CollisionObject) -> usize {
        let i = self.bodies.len();
        rb.id = i;
        let co_id = self.collision_world.add_collision_object(co);
        rb.link_and_set_collision_object(self.collision_world.collision_object_mut(co_id).unwrap());
        self.bodies.push(rb);

        i
    }

    pub fn rigidbody_ref(&self, id: usize) -> &RigidBody {
        &self.bodies[id]
    }

    pub fn rigidbody_mut(&mut self, id: usize) -> &mut RigidBody {
        &mut self.bodies[id]
    }

    pub fn solve_contact_manifolds(&mut self) {
        // comme si les deux objets etait mobiles
        println!(
            "solve_contact_manifolds {:?}",
            self.collision_world.contact_manifolds.len()
        );
        for cm in self.collision_world.contact_manifolds.iter() {
            let rb1_id = self
                .collision_world
                .collision_object_ref(cm.id_collision_object_a)
                .unwrap()
                .id_rigid_body
                .unwrap();
            let rb2_id = self
                .collision_world
                .collision_object_ref(cm.id_collision_object_b)
                .unwrap()
                .id_rigid_body
                .unwrap();

            let normal = cm.contact_infos.normal_a_to_b;
            match (self.bodies[rb1_id].is_static, self.bodies[rb2_id].is_static) {
                (false, false) => {
                    println!("both mving");

                    for p in &cm.contact_infos.points {
                        let r1 = p - &self.bodies[rb1_id].transform.position();
                        let r2 = p - &self.bodies[rb2_id].transform.position();
                        let (j1, j2) = collision_solver::compute_amps_moving_objects(
                            &self.bodies[rb1_id],
                            &self.bodies[rb2_id],
                            &r1,
                            &r2,
                            &normal,
                        );
                        self.bodies[rb1_id].apply_linear_impulse(-normal * j1);
                        self.bodies[rb2_id].apply_linear_impulse(normal * j2);
                        self.bodies[rb1_id].apply_angular_impulse(cross(&r1, &(-normal * j1)));
                        self.bodies[rb2_id].apply_angular_impulse(cross(&r2, &(normal * j2)));
                    }
                }
                (false, true) => {
                    println!("First mving");
                    for p in &cm.contact_infos.points {
                        let r1 = p - &self.bodies[rb1_id].transform.position();
                        // let r2 = p - &self.bodies[rb2_id].transform.position();

                        let j = collision_solver::compute_amps_one_moving_object(
                            &self.bodies[rb1_id],
                            &r1,
                            &normal,
                        );
                        println!("amp is {:?} normal={:?}", j, normal);

                        self.bodies[rb1_id].apply_linear_impulse(normal * j);
                        // self.bodies[rb2_id].apply_linear_impulse(-normal * j2);
                        self.bodies[rb1_id].apply_angular_impulse(cross(&r1, &(normal * j)));
                        // self.bodies[rb2_id].apply_angular_impulse(cross(&r2, &(-normal * j2)));
                    }
                }
                (true, false) => {
                    println!("second mving");

                    for p in &cm.contact_infos.points {
                        // let r1 = p - &self.bodies[rb1_id].transform.position();
                        let r2 = p - &self.bodies[rb2_id].transform.position();

                        let j = collision_solver::compute_amps_one_moving_object(
                            &self.bodies[rb2_id],
                            &r2,
                            &normal,
                        );
                        println!("amp is {:?} normal={:?}", j, normal);
                        // self.bodies[rb1_id].apply_linear_impulse(normal * j1);
                        self.bodies[rb2_id].apply_linear_impulse(normal * j);
                        // self.bodies[rb1_id].apply_angular_impulse(cross(&r1, &(normal * j1)));
                        self.bodies[rb2_id].apply_angular_impulse(cross(&r2, &(normal * j)));
                    }
                }
                (true, true) => {
                    println!("None mving");
                }
            };
        }
    }

    pub fn discrete_step(&mut self) {
        // 1. On fait avancer les collisions
        self.collision_world.step();

        // 2. On calcul et on applique les impulsions
        self.solve_contact_manifolds();

        self.collision_world.clear_manifold();

        // 3. integre les nouveaux états( vitesses lin ang, position, orientation) et update inertia tensor on rigidbodies
        for rb in self.bodies.iter_mut() {
            if !rb.is_static {
                rb.apply_gravity();
                rb.integrate_velocities(self.time_step);
                rb.integrate_state(self.time_step);

                // 4. Mettre à jour la position pour les shapes des collisions object associés aux rigid bodies
                self.collision_world.update_transform_collision_object(
                    rb.collision_object_id(),
                    rb.transform.clone(),
                );
                // println!(
                //     "{:?} {:?} \n",
                //     rb.transform.position(),
                //     self.collision_world
                //         .collision_object_ref(rb.collision_object_id)
                //         .unwrap()
                //         .shape
                //         .get_transform()
                //         .position()
                // );
            }
        }
    }
}
