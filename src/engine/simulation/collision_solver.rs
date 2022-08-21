use super::consts;
use super::RigidBody;
use super::SimulationWorld;
use crate::engine::contact_algorithms::ContactInformations;
use crate::math::{math_essentials::*, Quaternion};

pub struct ContactSolver<'a> {
    sim: &'a mut SimulationWorld,
    // penetration
}

impl<'a> ContactSolver<'a> {
    pub fn new(simulation: &'a mut SimulationWorld) -> ContactSolver {
        ContactSolver { sim: simulation }
    }

    pub fn pre_compute(&mut self) {
        
    }
}

/**
 * Return a triplet, useful in the penetration_resolution() stage
 */
pub fn rotation_per_unit_impulse(
    rigidbody: &RigidBody,
    point: &P3,
    normal: &Vec3,
) -> (Vec3, Vec3, Vec3) {
    let torque_per_unit_impulse = cross(&point, &normal); // normal is the unit
    let rotation_per_unit_impulse_v = rigidbody.inv_inertia_tensor() * &torque_per_unit_impulse;
    let angular_velocity_per_unit_impulse = cross(&rotation_per_unit_impulse_v, &point);

    (
        torque_per_unit_impulse,
        rotation_per_unit_impulse_v,
        angular_velocity_per_unit_impulse,
    )
}

pub fn velocity_at_point(rigidbody: &RigidBody, relative_contact_point: &P3) -> Vec3 {
    rigidbody.linear_velocity + cross(&rigidbody.angular_velocity, &relative_contact_point)
}

/**
 * Clear source =
 * https://en.wikipedia.org/wiki/Collision_response
 */
pub fn velocity_resolution_two_moving(
    bodies: &mut Vec<RigidBody>,
    rb1_id: usize,
    rb2_id: usize,
    p: &P3,
    contact_infos: &ContactInformations,
) {
    let rb1 = &bodies[rb1_id];
    let rb2 = &bodies[rb2_id];
    let rb1_2_point = p - rb1.center_of_mass();
    let rb2_2_point = p - rb2.center_of_mass();
    let normal = &contact_infos.normal_a_to_b;
    let v1 = velocity_at_point(rb1, &rb1_2_point);
    let v2 = velocity_at_point(rb2, &rb2_2_point);
    let rel_velocity = v2 - v1;
    let rel_velocity_normal = dot(&rel_velocity, &normal);

    // si les deux objets s'eloignent l'un de l'autre alors pas d'impulsion
    if dot(&normal, &v1) < ZERO && dot(&normal, &v2) > ZERO {
        return;
    }

    let (_, _, rot_per_impulse_1) = rotation_per_unit_impulse(rb1, &p, &normal);
    let (_, _, rot_per_impulse_2) = rotation_per_unit_impulse(rb2, &p, &normal);

    let div =
        (rb1.inv_mass() + rb2.inv_mass()) + dot(&(rot_per_impulse_1 + rot_per_impulse_2), &normal);

    let t = rel_velocity_normal / div;

    let imp_a = -(ONE + rb1.restitution_coef) * t;
    let imp_b = -(ONE + rb2.restitution_coef) * t;
    let impulsion_vector_a = -normal * imp_a;
    let impulsion_vector_b = normal * imp_b;

    println!("1 linear_imp={:?} ", impulsion_vector_a);
    println!("2 linear_imp={:?} ", impulsion_vector_b);

    // drop the ref to mut data
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

/**
 *  rb is always the moving object
 *  and the normal must point from the moving object toward the static one,
 *  thus, we need to pass it directly, and not trough the contact_infos object
 * (because we change its direction)
 */
pub fn velocity_resolution_one_moving(
    rb: &mut RigidBody,
    p: &P3,
    normal: Vec3,
    penetration_distance: Real,
) {
    let center2point = p - rb.center_of_mass();
    let rel_velocity = velocity_at_point(rb, &center2point);
    let rel_velocity_normal = dot(&rel_velocity, &normal);
    println!("==resolve_collision_point_one_moving==");

    println!(
        "normal {:?} rel_velocity_normal {:?} ",
        normal, rel_velocity_normal
    );
    if rel_velocity_normal < ZERO {
        return;
    }

    let (_, _, rot_per_impulse) = rotation_per_unit_impulse(rb, &p, &normal);

    let div = rb.inv_mass() + dot(&rot_per_impulse, &normal);

    println!(
        "div={:?} rel_velocity={:?} rel_velo_normal={:?}",
        div, rel_velocity, rel_velocity_normal
    );
    let t = rel_velocity_normal / div;
    let impulsion = -(ONE + rb.restitution_coef) * t;
    let impulsion_vector = normal * impulsion;

    println!("linear_imp={:?} ", impulsion_vector);

    rb.apply_linear_impulse(impulsion_vector);
    rb.apply_angular_impulse(cross(
        &center2point,
        &(impulsion_vector * consts::ANGULAR_DAMPING),
    ));
}

pub fn limit_angular_displacement(
    mut linear_move: Real,
    mut angular_move: Real,
    relative_contact: &Vec3,
) -> (Real, Real) {
    let limit = consts::ANGULAR_PENETRATION_DISPLACEMENT_FACTOR * magnitude(relative_contact);

    if angular_move.abs() > limit {
        let total_move = linear_move + angular_move;

        if angular_move >= ZERO {
            angular_move = limit;
        } else {
            angular_move = -limit;
        }

        linear_move = total_move - angular_move;
    }

    (linear_move, angular_move)
}

/**
 * Source and most of the code :
 * http://www.r-5.org/files/books/computers/algo-list/realtime-3d/Ian_Millington-Game_Physics_Engine_Development-EN.pdf
 */
pub fn penetration_resolution_two_moving(
    bodies: &mut Vec<RigidBody>,
    rb1_id: usize,
    rb2_id: usize,
    p: &P3,
    contact_infos: &ContactInformations,
) {
    let normal = contact_infos.normal_a_to_b;

    let rel_contact_point_1 = p - bodies[rb1_id].center_of_mass();
    let rel_contact_point_2 = p - bodies[rb2_id].center_of_mass();

    // Computing the inertia, or what it takes to move a rigidbody by one unit
    // in this case, its not impulsion, but movement
    let (_, rotation_direction_1, angular_velocity_per_unit_impulse_1) =
        rotation_per_unit_impulse(&bodies[rb1_id], &rel_contact_point_1, &normal);

    // angular inertia per unit impulse in direction
    let angular_inertia_1 = dot(&angular_velocity_per_unit_impulse_1, &normal);
    let linear_inertia_1 = bodies[rb1_id].inv_mass();

    let (_, rotation_direction_2, angular_velocity_per_unit_impulse_2) =
        rotation_per_unit_impulse(&bodies[rb1_id], &rel_contact_point_2, &normal);

    // angular inertia per unit impulse in direction
    let angular_inertia_2 = dot(&angular_velocity_per_unit_impulse_2, &normal);
    let linear_inertia_2 = bodies[rb2_id].inv_mass();

    // Computing the movement
    let penetration = contact_infos.penetration_distance;
    let inv_total_inerta =
        ONE / (angular_inertia_1 + linear_inertia_1 + angular_inertia_2 + linear_inertia_2);
    let linear_move_1 = -penetration * linear_inertia_1 * inv_total_inerta;
    let linear_move_2 = penetration * linear_inertia_2 * inv_total_inerta;
    let angular_move_1 = -penetration * angular_inertia_1 * inv_total_inerta;
    let angular_move_2 = penetration * angular_inertia_2 * inv_total_inerta;

    let (linear_move_1, angular_move_1) =
        limit_angular_displacement(linear_move_1, angular_move_1, &rel_contact_point_1);
    let (linear_move_2, angular_move_2) =
        limit_angular_displacement(linear_move_2, angular_move_2, &rel_contact_point_2);

    // Applying the computed movement
    let rotation_1 = rotation_direction_1 * angular_move_1 / angular_inertia_1;
    let rotation_2 = rotation_direction_2 * angular_move_2 / angular_inertia_2;

    bodies[rb1_id].apply_displacement(normal * linear_move_1);
    bodies[rb2_id].apply_displacement(normal * linear_move_2);
    bodies[rb1_id].apply_rotation_vector(Quaternion::from_vec(&rotation_1));
    bodies[rb2_id].apply_rotation_vector(Quaternion::from_vec(&rotation_2));
}
/**
 * See velocity_resolution_one_moving() above
 */
pub fn penetration_resolution_one_moving(
    rb: &mut RigidBody,
    p: &P3,
    normal: Vec3,
    penetration_distance: Real,
) {
    let rel_contact_point = p - rb.center_of_mass();
    // Computing the inertia, or what it takes to move a rigidbody by one unit
    // in this case, its not impulsion, but movement
    let (_, rotation_direction, angular_velocity_per_unit_impulse) =
        rotation_per_unit_impulse(&rb, &rel_contact_point, &normal);

    // angular inertia per unit impulse in direction
    let angular_inertia = dot(&angular_velocity_per_unit_impulse, &normal);

    let linear_inertia = rb.inv_mass();

    // Computing the movement
    let penetration = penetration_distance;
    let inv_total_inerta = ONE / (angular_inertia + linear_inertia);
    let linear_move = -penetration * linear_inertia * inv_total_inerta;
    let angular_move = -penetration * angular_inertia * inv_total_inerta;

    let (linear_move, angular_move) =
        limit_angular_displacement(linear_move, angular_move, &rel_contact_point);

    // Applying the computed movement
    let rotation = rotation_direction * angular_move / angular_inertia;

    rb.apply_displacement(normal * linear_move);
    rb.apply_rotation_vector(Quaternion::from_vec(&rotation));
}
