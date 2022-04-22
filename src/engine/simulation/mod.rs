pub mod rigid_body;

pub use rigid_body::RigidBody;

use crate::engine::collision::{CollisionObject, CollisionWorld};
use crate::math::math_essentials::*;
struct SimulationWorld<'a> {
    bodies: Vec<RigidBody<'a>>,
    time_step: Real,
    collision_world: CollisionWorld,
}

impl<'a> SimulationWorld<'a> {
    pub fn add_rigidbody(&mut self, rb: RigidBody<'a>) -> usize {
        let i = self.bodies.len();
        self.bodies.push(rb);
        i
    }
    pub fn discrete_timestep(&mut self) {
        // 1. On fait avancer les collisions
        self.collision_world.step();

        // 2. On calcul et on applique les impulsions
        for contact_manifold in self.collision_world.contact_manifolds.iter() {
            // contact_manifold.
        }

        // 3. integre les nouveaux états( vitesses lin ang, position, orientation) et update inertia tensor on rigidbodies
        for rb in self.bodies.iter_mut() {
            rb.apply_gravity();
            rb.integrate_velocities(self.time_step);
            rb.integrate_state(self.time_step);
            rb.update_inertia_tensor();
            // /!\ 4. Mettre à jour la position pour les shapes des collisions object associés aux rigid bodies
            // moyen pour retrouver un collision_object a partir de son id qu'on connait car stocké dans rigid bodies
        }
    }
}
