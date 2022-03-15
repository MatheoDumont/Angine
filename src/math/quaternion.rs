use super::{helper, Mat3, Real, Vec3, ONE, TWO, ZERO};
use std::ops::{Div, Mul};

/**
 * https://perso.liris.cnrs.fr/alexandre.meyer/teaching/master_charanim/aPDF_COURS_M2/M2_1b_Quaternions
 */
#[derive(Copy, Clone, Debug)]
pub struct Quaternion {
    pub w: Real,
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

impl Quaternion {
    pub fn from_vec(v: Vec3) -> Quaternion {
        Quaternion {
            w: ZERO,
            x: v[0],
            y: v[1],
            z: v[2],
        }
    }

    pub fn to_vec(q: &Quaternion) -> Vec3 {
        Vec3::new(q.x, q.y, q.z)
    }

    pub fn from_angle_axis(rad: Real, axis: Vec3) -> Quaternion {
        let r2 = rad / TWO;
        let s = r2.sin();
        Quaternion {
            w: r2.cos(),
            x: axis[0] * s,
            y: axis[1] * s,
            z: axis[2] * s,
        }
    }
    /**
     * Apply a rotation around z axis, y axis then x axis, in that order
     */
    pub fn from_euler(x: Real, y: Real, z: Real) -> Quaternion {
        Quaternion::from_angle_axis(x, Vec3::right())
            * Quaternion::from_angle_axis(y, Vec3::up())
            * Quaternion::from_angle_axis(z, Vec3::forward())
    }

    /**
     * source: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
     * 18 multiplications, 12 addition/soustractions = 30 opérations
     */
    pub fn to_mat3(&self) -> Mat3 {
        let xx2 = TWO * self.x * self.x;
        let yy2 = TWO * self.y * self.y;
        let zz2 = TWO * self.z * self.z;
        let xy2 = TWO * self.x * self.y;
        let zw2 = TWO * self.z * self.w;
        let xz2 = TWO * self.x * self.z;
        let yw2 = TWO * self.y * self.w;
        let yz2 = TWO * self.y * self.z;
        let xw2 = TWO * self.x * self.w;

        Mat3::from_array([
            [ONE - yy2 - zz2, xy2 - zw2, xz2 + yw2],
            [xy2 + zw2, ONE - xx2 - zz2, yz2 - xw2],
            [xz2 - yw2, yz2 + xw2, ONE - xx2 - yy2],
        ])
    }

    pub fn conjugate(&self) -> Quaternion {
        Quaternion {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn squared_norm(&self) -> Real {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn norm(&self) -> Real {
        self.squared_norm().sqrt()
    }

    pub fn inverse(&self) -> Quaternion {
        self.conjugate() / self.squared_norm()
    }

    pub fn rotate(&self, v: Vec3) -> Vec3 {
        let q = self * &Quaternion::from_vec(v) * self.conjugate();
        Quaternion::to_vec(&q)
    }
}

impl Mul for Quaternion {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Quaternion {
            w: self.w * rhs.w - (self.x * rhs.x + self.y * rhs.y + self.z * rhs.z),
            x: (self.w * rhs.x + self.x * rhs.w) + (self.y * rhs.z - rhs.y * self.z),
            y: (self.w * rhs.y + self.y * rhs.w) + (self.z * rhs.x - rhs.z * self.x),
            z: (self.w * rhs.z + self.z * rhs.w) + (self.x * rhs.y - rhs.x * self.y),
        }
    }
}

impl Mul for &Quaternion {
    type Output = Quaternion;
    fn mul(self, rhs: Self) -> Self::Output {
        Quaternion {
            w: self.w * rhs.w - (self.x * rhs.x + self.y * rhs.y + self.z * rhs.z),
            x: (self.w * rhs.x + self.x * rhs.w) + (self.y * rhs.z - rhs.y * self.z),
            y: (self.w * rhs.y + self.y * rhs.w) + (self.z * rhs.x - rhs.z * self.x),
            z: (self.w * rhs.z + self.z * rhs.w) + (self.x * rhs.y - rhs.x * self.y),
        }
    }
}

impl Div<Real> for Quaternion {
    type Output = Self;

    fn div(self, rhs: Real) -> Self {
        Quaternion {
            w: self.w / rhs,
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Rotation;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_vec_mat_from_quat() {
        // rot around Z
        {
            let q = Quaternion::from_euler(
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(90.0),
            );

            let r = q.rotate(Vec3::new(ONE, ZERO, ZERO));
            assert_approx_eq!(r[0], ZERO);
            assert_approx_eq!(r[1], ONE);
            assert_approx_eq!(r[2], ZERO);
        }

        // rot around Y
        {
            let q = Quaternion::from_euler(
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(90.0),
                helper::angle_2_rad(ZERO),
            );

            let r = q.rotate(Vec3::new(ZERO, ZERO, ONE));
            assert_approx_eq!(r[0], ONE);
            assert_approx_eq!(r[1], ZERO);
            assert_approx_eq!(r[2], ZERO);
        }

        // rot around X
        {
            let q = Quaternion::from_euler(
                helper::angle_2_rad(90.0),
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(ZERO),
            );

            let r = q.rotate(Vec3::new(ZERO, ONE, ZERO));
            assert_approx_eq!(r[0], ZERO);
            assert_approx_eq!(r[1], ZERO);
            assert_approx_eq!(r[2], ONE);
        }

        // composed y z
        {
            let q = Quaternion::from_euler(
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(90.0),
                helper::angle_2_rad(45.0),
            );

            let r = q.rotate(Vec3::new(ZERO, ONE, ZERO));
            assert_approx_eq!(r[0], ZERO);
            assert_approx_eq!(r[1], 0.7071067);
            assert_approx_eq!(r[2], 0.7071067);
        }
        // test unity
        // {
        //     let q = Quaternion::from_euler(
        //         helper::angle_2_rad(-30.0),
        //         helper::angle_2_rad(35.0),
        //         helper::angle_2_rad(45.0),
        //     );
        //     println!("{:?}", q);
        //     let r = q.rotate(Vec3::new(ONE, ONE, -ONE));
        //     println!("{:?}", r);
        //     println!(" Matrix {:?}", q.to_mat3());
        //     println!("{:?}", q.to_mat3() * Vec3::new(ONE, ONE, -ONE));

        // }
        {
            let m = Rotation::composed(ZERO, helper::angle_2_rad(90.0), helper::angle_2_rad(45.0));
            let q = Quaternion::from_euler(
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(90.0),
                helper::angle_2_rad(45.0),
            );

            let r = q.rotate(Vec3::new(ZERO, ONE, ZERO));
            println!("{:?}", r);
            println!("{:?}", m * Vec3::new(ZERO, ONE, ZERO));
            assert_approx_eq!(r[0], ZERO);
            assert_approx_eq!(r[1], 0.7071067);
            assert_approx_eq!(r[2], 0.7071067);
        }
    }
}
