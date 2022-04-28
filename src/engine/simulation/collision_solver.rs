use super::RigidBody;
use crate::engine::collision::{CollisionObject, CollisionWorld};
use crate::engine::contact_algorithms::{ContactInformation, ContactManifold};
use crate::math::{math_essentials::*, Mat3};

pub fn compute_amps_moving_objects(
    rb1: &RigidBody,
    rb2: &RigidBody,
    rb1_center2collisionpoint: &Vec3,
    rb2_center2collisionpoint: &Vec3,
    normal: &Vec3,
) -> (Real, Real) {
    let div = (rb1.inv_mass() + rb2.inv_mass())
        + dot(
            &(cross(
                &(rb1.inv_inertia_tensor() * &cross(&rb1_center2collisionpoint, &normal)),
                &rb1_center2collisionpoint,
            ) + cross(
                &(rb2.inv_inertia_tensor() * &cross(&rb2_center2collisionpoint, &normal)),
                &rb2_center2collisionpoint,
            )),
            &normal,
        );

    let rel_velocity = (rb1.linear_velocity
        + cross(&rb1.angular_velocity, &rb1_center2collisionpoint))
        - (rb2.linear_velocity + cross(&rb2.angular_velocity, &rb2_center2collisionpoint));
    let rel_velocity_normal = dot(&rel_velocity, &normal);
    let t = rel_velocity_normal / div;

    (
        helper::max(-(ONE + rb1.restitution_coef) * t, ZERO), // j1
        helper::max(-(ONE + rb2.restitution_coef) * t, ZERO), // j2
    )
}

pub fn compute_amps_one_moving_object(
    rb: &RigidBody,
    rb_center2collisionpoint: &Vec3,
    normal: &Vec3,
) -> Real {
    let rb2_inv_inertia_tensor = Mat3::diag(Vec3::value(100.0));
    let div = (rb.inv_mass() + 1.0 / 1000.0)
        + dot(
            &(cross(
                &(rb.inv_inertia_tensor() * &cross(&rb_center2collisionpoint, &normal)),
                &rb_center2collisionpoint,
            ) + cross(
                &(rb2_inv_inertia_tensor * cross(&Directions::up(), &normal)),
                &Directions::up(),
            )),
            &normal,
        );
    let rel_velocity = rb.linear_velocity + cross(&rb.angular_velocity, &rb_center2collisionpoint);

    let rel_velocity_normal = dot(&rel_velocity, &normal);
    let t = rel_velocity_normal / div;

    helper::max(-(ONE + rb.restitution_coef) * t, ZERO) // j1

    // let div = rb.inv_mass
    //     + dot(
    //         &cross(
    //             &(rb.inv_inertia_tensor() * &cross(&center2collisionpoint, &normal)),
    //             &center2collisionpoint,
    //         ),
    //         &normal,
    //     );

    // let rel_velocity = rb.linear_velocity + cross(&rb.angular_velocity, &center2collisionpoint);

    // let rel_velocity_normal = dot(&rel_velocity, &normal);
    // helper::min(
    //     -(ONE + rb.restitution_coef) * rel_velocity_normal / div,
    //     ZERO,
    // )
}
