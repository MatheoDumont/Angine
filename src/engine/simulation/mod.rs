pub mod collision_solver;
pub mod consts;
pub mod rigid_body;

pub use rigid_body::RigidBody;

use crate::engine::collision::{CollisionObject, CollisionWorld};
use crate::engine::contact_algorithms::{ContactInformation, ContactManifold};
use crate::math::math_essentials::*;

pub struct SimulationWorld {
    bodies: Vec<RigidBody>,
    time_step: Real,
    pub collision_world: CollisionWorld,
}
impl SimulationWorld {
    /**
     * `time_step` is 1/60sec (consts::DELTA_TIME) if None
     */
    pub fn new(time_step: Option<Real>) -> SimulationWorld {
        let mut dt = consts::DELTA_TIME;
        if let Some(x) = time_step {
            dt = x;
        }
        SimulationWorld {
            bodies: Vec::new(),
            time_step: dt,
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

    pub fn pre_start_set_appart(&mut self) {
        self.collision_world.step();

        self.set_appart();

        self.collision_world.contact_manifolds.clear();
    }

    pub fn solve_contact_manifolds(&mut self) {
        println!("============================ solve_contact_manifolds =================================");

        println!(
            "solve_contact_manifolds {:?}",
            self.collision_world.contact_manifolds.len()
        );
        for cm in &self.collision_world.contact_manifolds {
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

            println!(
                "Points de collisions ({:?}) : ",
                cm.contact_infos.points.len(),
            );
            for p in &cm.contact_infos.points {
                println!("       {:?}", p);
            }
            println!(
                "normal{:?} distance{:?}",
                cm.contact_infos.normal_a_to_b, cm.contact_infos.penetration_distance
            );
            match (self.bodies[rb1_id].is_static, self.bodies[rb2_id].is_static) {
                (false, false) => {
                    println!("both mving");

                    for p in &cm.contact_infos.points {
                        collision_solver::resolve_collision(
                            &mut self.bodies,
                            rb1_id,
                            rb2_id,
                            &p,
                            &cm.contact_infos,
                        );
                    }
                }
                (false, true) => {
                    println!("First mving");
                    println!(
                        "static={:?} moving={:?}",
                        self.bodies[rb2_id].transform.translation,
                        self.bodies[rb1_id].transform.translation,
                    );

                    for p in &cm.contact_infos.points {
                        collision_solver::resolve_collision_point_one_moving(
                            &mut self.bodies[rb1_id],
                            &p,
                            cm.contact_infos.normal_a_to_b,
                            cm.contact_infos.penetration_distance,
                        );
                    }
                }
                (true, false) => {
                    println!("second mving");
                    println!(
                        "static={:?} moving={:?}",
                        self.bodies[rb2_id].transform.translation,
                        self.bodies[rb1_id].transform.translation,
                    );
                    for p in &cm.contact_infos.points {
                        collision_solver::resolve_collision_point_one_moving(
                            &mut self.bodies[rb2_id],
                            &p,
                            -cm.contact_infos.normal_a_to_b,
                            cm.contact_infos.penetration_distance,
                        );
                    }
                }
                (true, true) => {
                    // println!("None mving");
                }
            };
        }
    }

    pub fn discrete_step(&mut self) {
        // 1. On fait avancer les collisions
        self.collision_world.step();

        // 2. On calcul et on applique les impulsions
        let n_ite = 1; //5-8
        for _ in 0..n_ite {
            self.solve_contact_manifolds();
        }

        // 3. integre les nouveaux ??tats( vitesses lin ang, position, orientation) et update inertia tensor on rigidbodies
        for i in 0..self.bodies.len() {
            self.step_rigidbody(i);
        }

        // linear displacement
        self.displacement();
        self.collision_world.clear_manifold();
    }

    pub fn displacement(&mut self) {
        println!("DISPLACEMENT");
        for cm in &self.collision_world.contact_manifolds {
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
            let distance = cm.contact_infos.penetration_distance;
            let total_force = self.bodies[rb1_id].mass() + self.bodies[rb2_id].mass();

            let d = normal * (helper::max(distance.abs() - 0.01, ZERO) / total_force) * 0.0125;

            match (self.bodies[rb1_id].is_static, self.bodies[rb2_id].is_static) {
                (false, false) => {
                    let i_m = self.bodies[rb1_id].inv_mass();
                    self.bodies[rb1_id].apply_displacement(-d * i_m);
                    println!("B1 [id={:?}] {:?} ", self.bodies[rb1_id].id, -d * i_m);

                    let i_m = self.bodies[rb2_id].inv_mass();
                    self.bodies[rb2_id].apply_displacement(d * i_m);
                    println!("B2 [id={:?}] {:?} ", self.bodies[rb2_id].id, d * i_m);
                }
                (false, true) => {
                    let i_m = self.bodies[rb1_id].inv_mass();
                    self.bodies[rb1_id].apply_displacement(d * i_m);
                    println!("B1 [id={:?}] {:?} ", self.bodies[rb1_id].id, -d * i_m);
                }
                (true, false) => {
                    let i_m = self.bodies[rb2_id].inv_mass();
                    self.bodies[rb2_id].apply_displacement(d * i_m);
                    println!("B2 [id={:?}] {:?} ", self.bodies[rb2_id].id, d * i_m);
                }
                (true, true) => {
                    // println!("None mving");
                }
            }
        }
    }

    pub fn set_appart(&mut self) {
        for cm in &self.collision_world.contact_manifolds {
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
            let distance = cm.contact_infos.penetration_distance;
            let d = normal * distance;

            match (self.bodies[rb1_id].is_static, self.bodies[rb2_id].is_static) {
                (false, false) => {
                    self.bodies[rb1_id].apply_displacement(-d);

                    self.bodies[rb2_id].apply_displacement(d);
                }
                (false, true) => {
                    self.bodies[rb1_id].apply_displacement(d);
                }
                (true, false) => {
                    self.bodies[rb2_id].apply_displacement(d);
                }
                (true, true) => {
                    // println!("None mving");
                }
            }
        }
    }

    pub fn step_rigidbody(&mut self, id: usize) {
        let rb = &mut self.bodies[id];
        if !rb.is_static {
            rb.apply_gravity();
            rb.integrate_velocities(self.time_step);
            rb.integrate_state(self.time_step);

            // 4. Mettre ?? jour la position pour les shapes des collisions object associ??s aux rigid bodies
            self.collision_world
                .update_transform_collision_object(rb.collision_object_id(), rb.transform.clone());

            println!(
                "  [{:?}]  {:?} mass={:?}",
                rb.id,
                rb.linear_velocity,
                rb.mass()
            );
        }
    }
}
