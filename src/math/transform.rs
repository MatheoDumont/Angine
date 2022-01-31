use super::{Mat3, Real, Vec3, P3};
use std::ops::Mul;

/**
 * pitch is a radian
 */
pub fn Xrotation(pitch: Real) -> Mat3 {
    let zero = 0 as Real;
    let c = pitch.cos();
    let s = pitch.sin();
    Mat3::from_array([[1 as Real, zero, zero], [zero, c, s], [zero, -s, c]])
}
pub fn Yrotation(yaw: Real) -> Mat3 {
    let zero = 0 as Real;
    let c = yaw.cos();
    let s = yaw.sin();
    Mat3::from_array([[c, zero, -s], [zero, 1 as Real, zero], [s, zero, c]])
}
pub fn Zrotation(roll: Real) -> Mat3 {
    let zero = 0 as Real;
    let c = roll.cos();
    let s = roll.sin();
    Mat3::from_array([[c, s, zero], [-s, c, zero], [zero, zero, 1 as Real]])
}

pub fn rotation(x_axis_rad: Real, y_axis_rad: Real, z_axis_rad: Real) -> Mat3 {
    Zrotation(z_axis_rad) * Xrotation(x_axis_rad) * Yrotation(y_axis_rad)
}

pub fn rotation_axis_angle(normalized_axis: Vec3, rad: Real) -> Mat3 {
    let c = rad.cos();
    let s = rad.sin();
    let t = (1 as Real) - c;
    let x = normalized_axis.x;
    let y = normalized_axis.y;
    let z = normalized_axis.z;

    let op1 = t * x * y + s * z;
    let op2 = t * x * z + s * y;
    let op3 = t * y * z + s * x;

    Mat3::from_array([
        [t * x * x + c, op1, op2],
        [op1, t * y * y + c, op3],
        [op2, op3, t * z * z + c],
    ])
}
/**
 * La class Transform contient une Mat3 pour la rotation et un Vec3 pour la translation.
 * Elle permet de changer un point ou un vecteur de repère, passer du repère local vers monde par exemple.
 * Donc elle doit fournir les opérations nécessaires :
 *
 */
pub struct Transform {
    rotation: Mat3,
    translation: Vec3,
}

impl Transform {
    pub fn new(scaling_vec: Vec3, rotation_mat: Mat3, translation_vec: Vec3) -> Transform {
        Transform {
            rotation: Mat3::diag(scaling_vec) * rotation_mat,
            translation: translation_vec,
        }
    }

    pub fn identity() -> Transform {
        Transform {
            rotation: Mat3::identity(),
            translation: Vec3::zero(),
        }
    }

    pub fn scaling(scales: Vec3) -> Transform {
        Transform {
            rotation: Mat3::diag(scales),
            translation: Vec3::zero(),
        }
    }

    pub fn translation(t: Vec3) -> Transform {
        Transform {
            rotation: Mat3::identity(),
            translation: t,
        }
    }

    pub fn rotation(rot: Mat3) -> Transform {
        Transform {
            rotation: rot,
            translation: Vec3::zero(),
        }
    }

    pub fn transform(&self, point: &P3) -> P3 {
        &(&self.rotation * point) + &self.translation
    }
    pub fn transform_vec(&self, vec: &Vec3) -> Vec3 {
        &self.rotation * vec
    }
}

impl Mul for Transform {
    type Output = Self;

    fn mul(self, o: Self) -> Self {
        Transform {
            rotation: &self.rotation * &o.rotation,
            translation: &self.translation + &o.translation,
        }
    }
}
