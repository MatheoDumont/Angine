use crate::engine::collision::CollisionObject;
use crate::math::math_essentials::*;

pub struct RigidBody {
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    total_force: Vec3,
    total_torque: Vec3,
    translation_moving_axis: Vec3,
    rotation_moving_axis: Vec3,
    collision_object: CollisionObject,
    inv_inertia_tensor: Mat3,
    mass: Real,
    inv_mass: Real,
}

impl RigidBody {
    pub fn integrate_velocity(&self, dt: Real) {
        self.linear_velocity += self.total_force * self.inv_mass * dt;
        self.angular_velocity += self.inv_inertia_tensor * self.total_torque * dt;

        self.total_force = ZERO;
        self.total_torque = ZERO;
    }
    pub fn update_inertia_tensor(&self) {
        self.collision_object.shape.get
    }
    pub fn apply_central_force(&self, force: Vec3) {
        self.total_force += force * translation_moving_axis;
    }
    pub fn apply_torque(&self, torque: Vec3) {
        self.total_torque += torque * rotation_moving_axis;
    }

    pub fn apply_force(&self, force: &Vec3) {
        apply_central_force(force);
        // apply_torque(cross(&relative_centroid, &force));
        apply_torque(force * translation_moving_axis);
    }

    pub fn apply_gravity(&self) {
        self.apply_force(-Directions::up() * 9.81 * self.mass);
    }
    /**
     * an impulse is a force
     */
    pub fn apply_linear_impulse(&self, impulse: Vec3) {}
    pub fn apply_angular_impulse(&self, impulse: Vec3) {
        self.angular_velocity += self.inv_inertia_tensor * impulse * self.rotation_moving_axis;
    }
}
