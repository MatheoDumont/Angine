use super::{Mat3, Real, Vec3, ONE, P3, ZERO};

use std::ops::Mul;

/**
 * pitch is a radian
 */

pub struct Rotation;

impl Rotation {
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

    pub fn compose(x_axis_rad: Real, y_axis_rad: Real, z_axis_rad: Real) -> Mat3 {
        Rotation::Z(z_axis_rad) * Rotation::X(x_axis_rad) * Rotation::Y(y_axis_rad)
    }

    pub fn axis_angle(normalized_axis: Vec3, rad: Real) -> Mat3 {
        let c = rad.cos();
        let s = rad.sin();
        let t = (ONE) - c;
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
}

pub trait Directions<T> {
    fn up() -> T;
    fn down() -> T;
    fn forward() -> T;
    fn backward() -> T;
    fn left() -> T;
    fn right() -> T;
}

impl Directions<Vec3> for Vec3 {
    fn up() -> Vec3 {
        Vec3 {
            x: ZERO,
            y: ZERO,
            z: ONE,
        }
    }
    fn down() -> Vec3 {
        Vec3 {
            x: ZERO,
            y: ZERO,
            z: -ONE,
        }
    }
    fn forward() -> Vec3 {
        Vec3 {
            x: ZERO,
            y: ONE,
            z: ZERO,
        }
    }
    fn backward() -> Vec3 {
        Vec3 {
            x: ZERO,
            y: -ONE,
            z: ZERO,
        }
    }
    fn left() -> Vec3 {
        Vec3 {
            x: -ONE,
            y: ZERO,
            z: ZERO,
        }
    }
    fn right() -> Vec3 {
        Vec3 {
            x: ONE,
            y: ZERO,
            z: ZERO,
        }
    }
}

/**
 * La classe Transform contient une Mat3 pour la rotation, le scale, et un Vec3 pour la translation.
 * Elle permet de changer un point de repère ou de rotate un vecteur.
 *
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

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn rotation_all_axis_test() {
        let rad = std::f32::consts::FRAC_PI_2;
        // Z rotation
        {
            let v = Vec3::new(ONE, ZERO, ZERO);
            let rotated = Rotation::Z(rad) * v;
            // println!("{:?}", rotated);
            assert_approx_eq!(rotated.x, ZERO, 1.0e-6);
            assert_approx_eq!(rotated.y, ONE, 1.0e-6);
            assert_approx_eq!(rotated.z, ZERO, 1.0e-6);
        }
        // Y rotation
        {
            let v = Vec3::new(ZERO, ZERO, ONE);
            let rotated = Rotation::Y(rad) * v;
            // println!("{:?}", rotated);
            assert_approx_eq!(rotated.x, ONE, 1.0e-6);
            assert_approx_eq!(rotated.y, ZERO, 1.0e-6);
            assert_approx_eq!(rotated.z, ZERO, 1.0e-6);
        }
        // X rotation
        {
            let v = Vec3::new(ZERO, ONE, ZERO);
            let rotated = Rotation::X(rad) * v;
            // println!("{:?}", rotated);
            assert_approx_eq!(rotated.x, ZERO, 1.0e-6);
            assert_approx_eq!(rotated.y, ZERO, 1.0e-6);
            assert_approx_eq!(rotated.z, ONE, 1.0e-6);
        }
    }
}
