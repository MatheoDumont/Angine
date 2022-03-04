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

pub fn closest_to_p(p: &P3, pts: &Vec<P3>) -> usize {
    let mut closest_vertex = 0;
    let mut d = (&pts[closest_vertex] - p).norm_squared();
    for i in 1..pts.len() {
        let dd = (p - &pts[i]).norm_squared();
        if d > dd {
            d = dd;
            closest_vertex = i;
        }
    }
    closest_vertex
}

const RAD2ANGLECONST: Real = std::f32::consts::PI / (180 as Real);

pub fn angle_2_rad(angle: Real) -> Real {
    angle * RAD2ANGLECONST
}
pub fn rad_2_angle(rad: Real) -> Real {
    rad / RAD2ANGLECONST
}
