pub mod rigid_body;

pub use rigid_body::RigidBody;

struct SimulationWorld {
    bodies: vec<RigidBody>,
    time_step: Real,
}

impl SimulationWorld {
    pub fn discrete_timestep(&self) {
        // 1. On fait avancer les collisions
        // 2. On calcul les impulsions
        // 3. On applique les impulse
        // 4. integre les nouveaux états( vitesses, position, orientation)
    }
}
