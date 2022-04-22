use crate::engine::collision::CollisionObject;
use crate::math::{math_essentials::*, Mat3, Quaternion};

pub struct RigidBody<'a> {
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    total_force: Vec3,
    total_torque: Vec3,
    translation_moving_axis: Vec3,
    rotation_moving_axis: Vec3,
    collision_object: &'a CollisionObject,
    inv_inertia_tensor: Mat3,
    mass: Real,
    inv_mass: Real,
    transform: Transform,
}

impl<'a> RigidBody<'a> {
    pub fn new(
        transform: Transform,
        mass: Real,
        collision_object: &'a CollisionObject,
    ) -> RigidBody {
        RigidBody {
            linear_velocity: Vec3::zeros(),
            angular_velocity: Vec3::zeros(),
            total_force: Vec3::zeros(),
            total_torque: Vec3::zeros(),
            translation_moving_axis: Vec3::ones(),
            rotation_moving_axis: Vec3::ones(),
            collision_object,
            inv_inertia_tensor: Mat3::zero(),
            mass,
            inv_mass: ONE / mass,
            transform,
        }
    }
    pub fn integrate_velocities(&mut self, dt: Real) {
        self.linear_velocity += self.total_force * self.inv_mass * dt;
        self.angular_velocity += self.inv_inertia_tensor * self.total_torque * dt;

        self.total_force = Vec3::zeros();
        self.total_torque = Vec3::zeros();
    }

    /**
     * Intégration d'euler semi implicite
     * avec quaternion derivative (slide 23) :
     * https://docs.google.com/presentation/d/1-UqEzGEHdskq8blwNWqdgnmUDwZDPjlZUvg437z7XCM/edit#slide=id.g6438a1c3e_0_0
     */
    pub fn integrate_state(&mut self, dt: Real) {
        self.transform.translation += self.linear_velocity * dt;

        // Quaternion derivative
        let q = Quaternion::from_mat3(&self.transform.rotation);
        let new_q = q + Quaternion::from_vec(&self.angular_velocity) * q * 0.5 * dt;
        self.transform.rotation = new_q.to_mat3();
    }
    /**
     * Inertia tensor slide 44 :
     * https://perso.liris.cnrs.fr/florence.zara/Web/media/files/M2-Animation/Chap4-RigidBody.pdf
     */
    pub fn update_inertia_tensor(&mut self) {
        self.inv_inertia_tensor = &self.transform.rotation
            * self.collision_object.shape.inverse_inertia_matrix()
            * self.transform.rotation.inverse();
    }

    pub fn apply_central_force(&mut self, force: Vec3) {
        self.total_force += force * self.translation_moving_axis;
    }
    pub fn apply_torque(&mut self, torque: Vec3) {
        self.total_torque += torque * self.rotation_moving_axis;
    }

    pub fn apply_force(&mut self, force: Vec3) {
        self.apply_central_force(force);
        // apply_torque(cross(&relative_centroid, &force));
        self.apply_torque(force * self.translation_moving_axis);
    }

    pub fn apply_gravity(&mut self) {
        self.apply_force(-Directions::up() * 9.81 * self.mass);
    }
    /**
     * an impulse is a force
     */
    pub fn apply_linear_impulse(&mut self, impulse: Vec3) {}
    pub fn apply_angular_impulse(&mut self, impulse: Vec3) {
        self.angular_velocity += self.inv_inertia_tensor * impulse * self.rotation_moving_axis;
    }
}
