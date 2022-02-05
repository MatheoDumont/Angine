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
