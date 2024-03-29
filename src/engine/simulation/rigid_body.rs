use super::consts;
use crate::engine::collision::CollisionObject;
use crate::math::{math_essentials::*, Mat3, Quaternion};

pub struct RigidBody {
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub total_force: Vec3,
    pub total_torque: Vec3,
    pub translation_moving_axis: Vec3,
    pub rotation_moving_axis: Vec3,
    collision_object_id: Option<usize>,
    inertia_matrix: Mat3,
    inv_inertia_matrix: Mat3,
    inv_inertia_tensor: Mat3,
    mass: Real,
    inv_mass: Real,
    pub transform: Transform,
    pub id: usize,
    pub restitution_coef: Real,
    pub is_static: bool,
    center_of_mass: P3,
    local_center_of_mass: P3,
}

impl RigidBody {
    pub fn new(transform: Transform, mass: Real, is_static: bool) -> RigidBody {
        let mut translation_moving_axis = Vec3::ones();
        let mut rotation_moving_axis = Vec3::ones();
        let mut mass = mass;
        let mut inv_mass = ZERO;

        if mass != ZERO {
            inv_mass = 1.0 / mass;
        }
        if is_static {
            translation_moving_axis = Vec3::value(ZERO);
            rotation_moving_axis = Vec3::value(ZERO);
            mass = ZERO;
            inv_mass = ZERO;
        }

        RigidBody {
            linear_velocity: Vec3::zeros(),
            angular_velocity: Vec3::zeros(),
            total_force: Vec3::zeros(),
            total_torque: Vec3::zeros(),
            translation_moving_axis,
            rotation_moving_axis,
            collision_object_id: None,
            inertia_matrix: Mat3::zero(),
            inv_inertia_matrix: Mat3::zero(),
            inv_inertia_tensor: Mat3::zero(),
            mass,
            inv_mass,
            transform,
            id: 0,
            restitution_coef: 0.95,
            is_static,
            center_of_mass: transform.translation,
            local_center_of_mass: P3::origin(),
        }
    }
    pub fn center_of_mass(&self) -> &P3 {
        &self.center_of_mass
    }

    /**
     * Call only when *this* is linked with its collision object, i.e link_and_set_collision_object() has been called.
     */
    pub fn collision_object_id(&self) -> usize {
        self.collision_object_id.unwrap()
    }

    pub fn mass(&self) -> Real {
        self.mass
    }

    pub fn inv_mass(&self) -> Real {
        self.inv_mass
    }
    /**
     * setter for mass and inv_mass
     */
    pub fn set_mass(&mut self, mass: Real, co: &CollisionObject) {
        debug_assert!(mass != ZERO);
        self.mass = mass;
        self.inv_mass = 1.0 / self.mass;
        self.inertia_matrix = co.shape.compute_inertia_matrix(self.mass);
        self.inv_inertia_matrix = self.inertia_matrix.inverse();
    }

    pub fn inertia_tensor(&self) -> Mat3 {
        self.transform.rotation * self.inertia_matrix * self.transform.rotation.inverse()
    }
    pub fn inv_inertia_tensor(&self) -> &Mat3 {
        &self.inv_inertia_tensor
    }

    pub fn link_and_set_collision_object(&mut self, co: &mut CollisionObject) {
        co.rigidbody_id = Some(self.id);
        co.is_static = self.is_static;
        self.collision_object_id = Some(co.id);
        co.shape.set_transform(self.transform.clone());
        if !self.is_static {
            // self.local_center_of_mass = ...
            self.inertia_matrix = co.shape.compute_inertia_matrix(self.mass);
            self.inv_inertia_matrix = self.inertia_matrix.inverse();
        }
    }

    pub fn set_static(&mut self) {
        self.is_static = true;
        self.translation_moving_axis = Vec3::value(ZERO);
        self.rotation_moving_axis = Vec3::value(ZERO);
        self.mass = ZERO;
        self.inv_mass = ZERO;
    }

    pub fn integrate_velocities(&mut self, dt: Real) {
        let coef_air_resistance = 0.97;
        self.linear_velocity += self.total_force * self.inv_mass * dt * coef_air_resistance;
        self.angular_velocity += self.inv_inertia_tensor * self.total_torque * dt;
        self.total_force = Vec3::zeros();
        self.total_torque = Vec3::zeros();
    }

    /**
     * Intégration d'euler semi implicite
     * avec quaternion derivative (slide 23) :
     * https://docs.google.com/presentation/d/1-UqEzGEHdskq8blwNWqdgnmUDwZDPjlZUvg437z7XCM/edit#slide=id.g6438a1c3e_0_0
     *
     * Inertia tensor slide 44 :
     * https://perso.liris.cnrs.fr/florence.zara/Web/media/files/M2-Animation/Chap4-RigidBody.pdf
     */
    pub fn integrate_state(&mut self, dt: Real) {
        self.apply_displacement(self.linear_velocity * dt);
        self.apply_rotation_vector(Quaternion::from_vec(&self.angular_velocity) * dt);

        // update center of mass
        self.center_of_mass = self.local_center_of_mass + self.transform.translation;

        // update inverse inertia tensor
        self.inv_inertia_tensor =
            self.transform.rotation * (self.inv_inertia_matrix * self.transform.rotation.inverse());
    }

    pub fn apply_central_force(&mut self, force: Vec3) {
        self.total_force += force * self.translation_moving_axis;
    }
    pub fn apply_torque(&mut self, torque: Vec3) {
        self.total_torque += torque * self.rotation_moving_axis;
    }

    pub fn apply_force(&mut self, force: Vec3, point: P3) {
        self.apply_central_force(force);
        // apply_torque(cross(&relative_centroid, &force));
        self.apply_torque(cross(&(point - self.center_of_mass), &force));
    }

    pub fn apply_gravity(&mut self) {
        self.apply_central_force(consts::GRAVITY_VECTOR * self.mass);
    }
    /**
     * an impulse is a force
     */
    pub fn apply_linear_impulse(&mut self, impulse: Vec3) {
        self.linear_velocity += impulse * self.inv_mass * self.translation_moving_axis;
    }
    pub fn apply_angular_impulse(&mut self, impulse: Vec3) {
        self.angular_velocity += self.inv_inertia_tensor * impulse * self.rotation_moving_axis;
    }

    /**
     *  Useful to resolve linear contact penetration
     */
    pub fn apply_displacement(&mut self, translation: Vec3) {
        self.transform.translation += translation;
    }

    /**
     * Use to compute the next orientation, based on a rotation_vector
     * telling us how much to rotate in each direction(Quaternion derivative).
     *
     * Also Useful to resolve angular contact penetration
     */
    pub fn apply_rotation_vector(&mut self, quat_from_vec: Quaternion) {
        // Quaternion derivative
        let q = Quaternion::from_mat3(&self.transform.rotation);
        self.transform.rotation = q.quaternion_derivative(quat_from_vec).to_mat3();
    }
}
