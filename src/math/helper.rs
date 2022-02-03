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
