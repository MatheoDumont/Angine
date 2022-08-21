use crate::math::math_essentials::*;

pub const DELTA_TIME: Real = 1 as Real / 60 as Real;
pub const GRAVITY_ACCELERATION_CONST: Real = 9.81 as Real; // kg.m^-2
pub const GRAVITY_DIRECTION: Vector3 = Vector3 {
    data: [0 as Real, -1 as Real, 0 as Real],
};
pub const GRAVITY_VECTOR: Vector3 = Vector3 {
    data: [0 as Real, -GRAVITY_ACCELERATION_CONST, 0 as Real],
};
pub const GRAVITY_INTEGRATED: Real = GRAVITY_ACCELERATION_CONST * DELTA_TIME;

pub const ANGULAR_DAMPING: Real = 0.9;

pub const ANGULAR_PENETRATION_DISPLACEMENT_FACTOR: Real = 0.2;
