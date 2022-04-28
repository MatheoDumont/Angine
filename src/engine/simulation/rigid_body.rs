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
}

impl RigidBody {
    pub fn new(transform: Transform, mass: Real, is_static: bool) -> RigidBody {
        let mut translation_moving_axis = Vec3::ones();
        let mut rotation_moving_axis = Vec3::ones();
        if is_static {
            translation_moving_axis = Vec3::value(ZERO);
            rotation_moving_axis = Vec3::value(ZERO);
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
            inv_mass: ONE / mass,
            transform,
            id: 0,
            restitution_coef: ONE, // totalement elastique
            is_static,
        }
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
        self.mass = mass;
        self.inv_mass = 1.0 / self.mass;
        self.inertia_matrix = co.shape.compute_inertia_matrix(self.mass);
        self.inv_inertia_matrix = self.inertia_matrix.inverse();
    }

    pub fn inertia_tensor(&self) -> Mat3 {
        self.transform.rotation * self.inertia_matrix * self.transform.rotation.inverse()
    }
    pub fn inv_inertia_tensor(&self) -> &Mat3 {
        &self.inv_inertia_tensor()
    }

    pub fn link_and_set_collision_object(&mut self, co: &mut CollisionObject) {
        co.id_rigid_body = Some(self.id);
        co.is_static = self.is_static;
        self.collision_object_id = Some(co.id);
        co.shape.set_transform(self.transform.clone());
        if !self.is_static {
            self.inertia_matrix = co.shape.compute_inertia_matrix(self.mass);
            self.inv_inertia_matrix = self.inertia_matrix.inverse();
        }
    }

    pub fn set_static(&mut self) {
        self.is_static = true;
        self.translation_moving_axis = Vec3::value(ZERO);
        self.rotation_moving_axis = Vec3::value(ZERO);
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
     *
     * Inertia tensor slide 44 :
     * https://perso.liris.cnrs.fr/florence.zara/Web/media/files/M2-Animation/Chap4-RigidBody.pdf
     */
    pub fn integrate_state(&mut self, dt: Real) {
        self.transform.translation += self.linear_velocity * dt;

        // Quaternion derivative
        let q = Quaternion::from_mat3(&self.transform.rotation);
        let new_q = q + Quaternion::from_vec(&self.angular_velocity) * q * 0.5 * dt;
        self.transform.rotation = new_q.to_mat3();

        // update inverse inertia tensor
        self.inv_inertia_tensor =
            self.transform.rotation * self.inv_inertia_matrix * self.transform.rotation.inverse();
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
    pub fn apply_linear_impulse(&mut self, impulse: Vec3) {
        self.linear_velocity += impulse * self.inv_mass * self.translation_moving_axis;
    }
    pub fn apply_angular_impulse(&mut self, impulse: Vec3) {
        self.angular_velocity += self.inv_inertia_tensor * impulse * self.rotation_moving_axis;
    }
}
