pub mod rigid_body;

pub use rigid_body::RigidBody;

use crate::engine::collision::{CollisionObject, CollisionWorld};
use crate::engine::contact_algorithms::{ContactInformation, ContactManifold};
use crate::math::math_essentials::*;
struct SimulationWorld {
    bodies: Vec<RigidBody>,
    time_step: Real,
    collision_world: CollisionWorld,
}
use std::rc::Rc;
impl SimulationWorld {
    pub fn add_rigidbody(&mut self, mut rb: RigidBody, mut co: CollisionObject) -> usize {
        let i = self.bodies.len();
        rb.id = i;
        co.id_rigid_body = Some(i);
        self.bodies.push(rb);
        self.collision_world.add_collision_object(co);
        i
    }
    pub fn solve_contact_manifolds(&mut self) {
        // comme si les deux objets etait mobiles
        for cm in self.collision_world.contact_manifolds.iter() {
            let rb1_id = self
                .collision_world
                .get_collision_object(cm.id_collision_object_a)
                .unwrap()
                .as_ref()
                .id_rigid_body
                .unwrap();
            let rb2_id = self
                .collision_world
                .get_collision_object(cm.id_collision_object_b)
                .unwrap()
                .as_ref()
                .id_rigid_body
                .unwrap();

            let normal = cm.contact_infos.normal_a_to_b;

            for p in &cm.contact_infos.points {
                let r1 = p - &self.bodies[rb1_id].transform.position();
                let r2 = p - &self.bodies[rb2_id].transform.position();

                let j1;
                let j2;
                // new scope for now, later in a function
                {
                    let rb1 = &self.bodies[rb1_id];
                    let rb2 = &self.bodies[rb2_id];

                    let div = (rb1.inv_mass + rb2.inv_mass)
                        + dot(
                            &(cross(&(rb1.inv_inertia_tensor * cross(&r1, &normal)), &r1)
                                + cross(&(rb2.inv_inertia_tensor * cross(&r2, &normal)), &r2)),
                            &normal,
                        );

                    let rel_velocity = (rb1.linear_velocity + cross(&rb1.angular_velocity, &r1))
                        - (rb2.linear_velocity + cross(&rb2.angular_velocity, &r2));
                    let rel_velocity_normal = dot(&rel_velocity, &normal);
                    let t = rel_velocity_normal / div;

                    j1 = helper::min(-(ONE + rb1.restitution_coef) * t, ZERO);
                    j2 = helper::min(-(ONE + rb2.restitution_coef) * t, ZERO);
                }

                self.bodies[rb1_id].apply_linear_impulse(normal * j1);
                self.bodies[rb2_id].apply_linear_impulse(-normal * j2);

                self.bodies[rb1_id].apply_angular_impulse(cross(&r1, &(normal * j1)));
                self.bodies[rb2_id].apply_angular_impulse(cross(&r2, &(-normal * j2)));
            }
        }
    }

    pub fn discrete_timestep(&mut self) {
        // 1. On fait avancer les collisions
        self.collision_world.step();

        // 2. On calcul et on applique les impulsions
        self.solve_contact_manifolds();

        // 3. integre les nouveaux états( vitesses lin ang, position, orientation) et update inertia tensor on rigidbodies
        for rb in self.bodies.iter_mut() {
            rb.apply_gravity();
            rb.integrate_velocities(self.time_step);
            rb.integrate_state(self.time_step);
            let rc_co = match self
                .collision_world
                .get_collision_object(rb.collision_object_id)
            {
                Ok(x) => x,
                Err(str) => panic!("{}", str),
            };
            rb.update_inertia_tensor(Rc::as_ref(rc_co).shape.inverse_inertia_matrix().clone());
            // 4. Mettre à jour la position pour les shapes des collisions object associés aux rigid bodies
            self.collision_world
                .update_transform_collision_object(rb.collision_object_id, rb.transform.clone());
        }
    }
}
