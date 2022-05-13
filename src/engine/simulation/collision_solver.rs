use super::consts;
use super::RigidBody;
use crate::engine::contact_algorithms::ContactInformation;
use crate::math::math_essentials::*;

pub fn resolve_collision(
    bodies: &mut Vec<RigidBody>,
    rb1_id: usize,
    rb2_id: usize,
    p: &P3,
    contact_infos: &ContactInformation,
) {
    let rb1 = &bodies[rb1_id];
    let rb2 = &bodies[rb2_id];
    let rb1_2_point = p - rb1.center_of_mass();
    let rb2_2_point = p - rb2.center_of_mass();
    let normal = &contact_infos.normal_a_to_b;
    println!(
        "normal{:?} distance{:?} p{:?}",
        normal, contact_infos.penetration_distance, p
    );
    let v1 = rb1.linear_velocity + cross(&rb1.angular_velocity, &rb1_2_point);
    let v2 = rb2.linear_velocity + cross(&rb2.angular_velocity, &rb2_2_point);
    let rel_velocity = v1 - v2;
    let rel_velocity_normal = dot(&rel_velocity, &normal);
    // si les deux objets s'eloignent l'un de l'autre alors pas d'impulsion
    if dot(&normal, &v1) < ZERO && dot(&normal, &v2) > ZERO {
        return;
    }
    let div = (rb1.inv_mass() + rb2.inv_mass())
        + dot(
            &(cross(
                &(rb1.inv_inertia_tensor() * &cross(&rb1_2_point, &normal)),
                &rb1_2_point,
            ) + cross(
                &(rb2.inv_inertia_tensor() * &cross(&rb2_2_point, &normal)),
                &rb2_2_point,
            )),
            &normal,
        );

    let t = rel_velocity_normal / div;
    // set them appart if intersect but no displacement shared along the normal
    if t == ZERO {
        bodies[rb1_id].apply_displacement(-normal * contact_infos.penetration_distance);
        bodies[rb2_id].apply_displacement(normal * contact_infos.penetration_distance);
    } else {
        let impulsion_vector_a = -normal * (ONE + rb1.restitution_coef) * t;
        let impulsion_vector_b = normal * (ONE + rb2.restitution_coef) * t;

        println!("1 linear_imp={:?} ", impulsion_vector_a);
        println!("2 linear_imp={:?} ", impulsion_vector_b);

        drop(rb1);
        drop(rb2);

        bodies[rb1_id].apply_linear_impulse(impulsion_vector_a);
        bodies[rb1_id].apply_angular_impulse(cross(
            &rb1_2_point,
            &(impulsion_vector_a * consts::ANGULAR_DAMPING),
        ));

        bodies[rb2_id].apply_linear_impulse(impulsion_vector_b);
        bodies[rb2_id].apply_angular_impulse(cross(
            &rb2_2_point,
            &(impulsion_vector_b * consts::ANGULAR_DAMPING),
        ));
    }
}

// rb is always the moving object
// the normal must point from the moving toward the static object
pub fn resolve_collision_point_one_moving(
    rb: &mut RigidBody,
    p: &P3,
    normal: Vec3,
    penetration_distance: Real,
) {
    let center2point = p - rb.center_of_mass();
    let rel_velocity = rb.linear_velocity + cross(&rb.angular_velocity, &center2point);
    let rel_velocity_normal = dot(&rel_velocity, &normal);

    if dot(&rel_velocity, &normal) > ZERO {
        return;
    }
    let div = rb.inv_mass()
        + dot(
            &cross(
                &(rb.inv_inertia_tensor() * &cross(&center2point, &normal)),
                &center2point,
            ),
            &normal,
        );

    println!(
        "div={:?} rel_velocity={:?} rel_velo_normal={:?}",
        div, rel_velocity, rel_velocity_normal
    );
    let t = rel_velocity_normal / div;
    if t == ZERO {
        rb.apply_displacement(normal * penetration_distance);
    } else {
        let impulsion = -(ONE + rb.restitution_coef) * t;
        let impulsion_vector = normal * impulsion;

        println!("linear_imp={:?} ", impulsion_vector);

        rb.apply_linear_impulse(impulsion_vector);
        rb.apply_angular_impulse(cross(
            &center2point,
            &(impulsion_vector * consts::ANGULAR_DAMPING),
        ));
    }
}
