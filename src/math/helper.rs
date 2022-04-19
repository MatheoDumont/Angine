use super::{Real, Vec3, Vector3, ONE, P3, ZERO};

pub fn clamp(x: Real, min: Real, max: Real) -> Real {
    debug_assert!(min < max);

    if x < min {
        min
    } else if x > max {
        max
    } else {
        x
    }
}

pub fn clamp_vector(x: Vector3, min: &Vector3, max: &Vector3) -> Vector3 {
    Vector3::new(
        clamp(x.x(), min.x(), max.x()),
        clamp(x.y(), min.y(), max.y()),
        clamp(x.z(), min.z(), max.z()),
    )
}
/**
 * Clamp x between t1 and t2 without knowing which one is the max and the min
 */
pub fn clamp_vector_between(x: Vector3, t1: &Vector3, t2: &Vector3) -> Vector3 {
    Vector3::new(
        clamp(x.x(), min(t1.x(), t2.x()), max(t1.x(), t2.x())),
        clamp(x.y(), min(t1.y(), t2.y()), max(t1.y(), t2.y())),
        clamp(x.z(), min(t1.z(), t2.z()), max(t1.z(), t2.z())),
    )
}

pub fn sign(x: Real) -> Real {
    if x > ZERO {
        ONE
    } else {
        -ONE
    }
}

pub fn min(f1: Real, f2: Real) -> Real {
    if f1 < f2 {
        f1
    } else {
        f2
    }
}

pub fn max(f1: Real, f2: Real) -> Real {
    if f1 > f2 {
        f1
    } else {
        f2
    }
}

const RAD2ANGLECONST: Real = std::f32::consts::PI / (180 as Real);

pub fn angle_2_rad(angle: Real) -> Real {
    angle * RAD2ANGLECONST
}
pub fn rad_2_angle(rad: Real) -> Real {
    rad / RAD2ANGLECONST
}

pub fn vect_angles(v: &Vec3) -> Vec3 {
    Vec3::new(rad_2_angle(v[0]), rad_2_angle(v[1]), rad_2_angle(v[2]))
}

/**
 * Round x to the n_th decimal
 */
pub fn round_n_decimal(x: Real, n: i32) -> Real {
    let div = 10.0_f32.powi(n);
    (x * div).round() / div
}

/**
 * Round Vector with round_n_decimal on each data
 */
pub fn round_n_decimal_vector(x: &Vector3, n: i32) -> Vector3 {
    Vector3::new(
        round_n_decimal(x.x(), n),
        round_n_decimal(x.y(), n),
        round_n_decimal(x.z(), n),
    )
}
