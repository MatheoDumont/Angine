use super::{Mat3, Quaternion, Real, Vec3, ONE, P3, ZERO};

use std::ops::Mul;

pub struct Rotation;

impl Rotation {
    /**
     * All angles are in radian
     */
    pub fn X(pitch: Real) -> Mat3 {
        let c = pitch.cos();
        let s = pitch.sin();
        Mat3::from_array([[ONE, ZERO, ZERO], [ZERO, c, -s], [ZERO, s, c]])
    }
    pub fn Y(yaw: Real) -> Mat3 {
        let c = yaw.cos();
        let s = yaw.sin();
        Mat3::from_array([[c, ZERO, s], [ZERO, ONE, ZERO], [-s, ZERO, c]])
    }
    pub fn Z(roll: Real) -> Mat3 {
        let c = roll.cos();
        let s = roll.sin();
        Mat3::from_array([[c, -s, ZERO], [s, c, ZERO], [ZERO, ZERO, ONE]])
    }

    pub fn composed(x_axis_rad: Real, y_axis_rad: Real, z_axis_rad: Real) -> Mat3 {
        // Rotation::Z(z_axis_rad) * Rotation::X(x_axis_rad) * Rotation::Y(y_axis_rad)
        Quaternion::from_euler_rads(x_axis_rad, y_axis_rad, z_axis_rad).to_mat3()
    }
}

/**
 * La classe Transform contient une Mat3 pour la rotation, le scale, et un Vec3 pour la translation.
 * Elle permet de changer un point de repère ou de rotate un vecteur.
 *
 *  
 */
pub struct Transform {
    pub rotation: Mat3,
    pub translation: Vec3,
}
impl Transform {
    pub fn new(scaling_vec: Vec3, rotation_mat: Mat3, translation_vec: Vec3) -> Transform {
        let mut r = rotation_mat;
        *r.at_mut(0, 0) *= scaling_vec[0];
        *r.at_mut(1, 1) *= scaling_vec[1];
        *r.at_mut(2, 2) *= scaling_vec[2];
        Transform {
            rotation: r,
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

    pub fn position(&self) -> P3 {
        P3::from(self.translation)
    }
}

impl Mul for Transform {
    type Output = Self;

    fn mul(self, o: Self) -> Self {
        Transform {
            rotation: self.rotation * o.rotation,
            translation: self.translation + o.translation,
        }
    }
}

impl Mul for &Transform {
    type Output = Transform;

    fn mul(self, o: Self) -> Self::Output {
        Transform {
            rotation: &self.rotation * &o.rotation,
            translation: &self.translation + &o.translation,
        }
    }
}
