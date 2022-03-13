use super::{Real, Vec3, ONE, P3, ZERO};

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
