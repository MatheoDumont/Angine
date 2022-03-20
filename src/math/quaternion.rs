use super::{helper, Directions, Mat3, Real, Vec3, ONE, TWO, ZERO};
use std::ops::{Div, Mul};

/**
 * https://perso.liris.cnrs.fr/alexandre.meyer/teaching/master_charanim/aPDF_COURS_M2/M2_1b_Quaternions
 * https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
 *
 * 56 opérations pour transformer un vec (36 selon wikipédia, j'ai pas opti la fn rotate() )
 * 15 en tout pour transformer un vec par une matrice,
 * pour obtenir une matrice à partir d'un quaternion = 30 opérations,
 * 30+15 = 45
 * donc vaut mieux obtenir une mat pour transformer un point, voir plusieurs,
 * et garder les quaternions pour composer des rotations.
 */

#[derive(Copy, Clone, Debug)]
pub struct Quaternion {
    pub w: Real,
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

impl Quaternion {
    pub fn from_vec(v: &Vec3) -> Quaternion {
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

    pub fn from_rad_axis(rad: Real, axis: Vec3) -> Quaternion {
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
     * Returns a rotation that rotates z degrees around the z axis,
     * x degrees around the x axis, and y degrees around the y axis (in that order).
     */
    pub fn from_euler_angles(x: Real, y: Real, z: Real) -> Quaternion {
        Quaternion::from_rad_axis(helper::angle_2_rad(y), Directions::up())
            * Quaternion::from_rad_axis(helper::angle_2_rad(x), Directions::right())
            * Quaternion::from_rad_axis(helper::angle_2_rad(z), Directions::forward())
        // yaw = z, pitch = y, roll = x
        // let rad1 = x * 0.5;
        // let rad2 = y * 0.5;
        // let rad3 = z * 0.5;

        // let c1 = rad1.cos();
        // let s1 = rad1.sin();
        // let c2 = rad2.cos();
        // let s2 = rad2.sin();
        // let c3 = rad3.cos();
        // let s3 = rad3.sin();

        // // Quaternion {
        // //     w: c1 * c2 * c3 - s1 * s2 * s3,
        // //     x: c1 * s2 * s3 + s1 * c2 * c3,
        // //     y: c1 * s2 * c3 - s1 * c2 * s3,
        // //     z: c1 * c2 * s3 + s1 * s2 * c3,
        // // }
        // let c1c2 = c1 * c2;
        // let s1s2 = s1 * s2;
        // let c1s2 = c1 * s2;
        // let s1c2 = s1 * c2;
        // Quaternion {
        //     w: c1c2 * c3 - s1s2 * s3,
        //     x: c1s2 * s3 + s1c2 * c3,
        //     y: c1s2 * c3 - s1c2 * s3,
        //     z: c1c2 * s3 + s1s2 * c3,
        // }
    }

    pub fn from_euler_rads(x_rad: Real, y_rad: Real, z_rad: Real) -> Quaternion {
        Quaternion::from_rad_axis(x_rad, Directions::right())
            * (Quaternion::from_rad_axis(y_rad, Directions::up())
                * Quaternion::from_rad_axis(z_rad, Directions::forward()))
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
    /**
     * 56 opérations pour rotate un vec
     */
    pub fn rotate(&self, v: &Vec3) -> Vec3 {
        let q = self * &Quaternion::from_vec(v) * self.conjugate();
        Quaternion::to_vec(&q)
    }
}

/**
 * 16 multiplications + 12 additioins/soustractions = 28 opérations
 */
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
    use crate::math::RotationMatrix;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_vec_mat_from_quat() {
        // rot around Z
        {
            let q = Quaternion::from_euler_rads(
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(90.0),
            );

            let r = q.rotate(&Vec3::new(ONE, ZERO, ZERO));
            assert_approx_eq!(r[0], ZERO);
            assert_approx_eq!(r[1], ONE);
            assert_approx_eq!(r[2], ZERO);
        }

        // rot around Y
        {
            let q = Quaternion::from_euler_rads(
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(90.0),
                helper::angle_2_rad(ZERO),
            );

            let r = q.rotate(&Vec3::new(ZERO, ZERO, ONE));
            assert_approx_eq!(r[0], ONE);
            assert_approx_eq!(r[1], ZERO);
            assert_approx_eq!(r[2], ZERO);
        }

        // rot around X
        {
            let q = Quaternion::from_euler_rads(
                helper::angle_2_rad(90.0),
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(ZERO),
            );

            let r = q.rotate(&Vec3::new(ZERO, ONE, ZERO));
            assert_approx_eq!(r[0], ZERO);
            assert_approx_eq!(r[1], ZERO);
            assert_approx_eq!(r[2], ONE);
        }

        // composed y z
        {
            let q = Quaternion::from_euler_rads(
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(90.0),
                helper::angle_2_rad(45.0),
            );

            let r = q.rotate(&Vec3::new(ZERO, ONE, ZERO));
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
    }
    #[test]
    fn test_diff_rotation_and_quat() {
        let x_angle = -30.0;
        let y_angle = 35.0;
        let z_angle = 45.0;

        let v = Vec3::new(ONE, ONE, ONE);
        let m = RotationMatrix::composed(
            helper::angle_2_rad(x_angle),
            helper::angle_2_rad(y_angle),
            helper::angle_2_rad(z_angle),
        );
        // z then y
        let q = Quaternion::from_euler_angles(x_angle, y_angle, z_angle);
        // let q = Quaternion {
        //     w: 0.8213125,
        //     x: -0.1168965,
        //     y: 0.3628112,
        //     z: 0.4244396,
        // };

        println!("norm of q {:?}, q: {:?} ", q.norm(), q);
        // y then z
        // let qq = Quaternion::from_rad_axis(helper::angle_2_rad(z_angle), Vec3::forward())
        //     * Quaternion::from_rad_axis(helper::angle_2_rad(y_angle), Vec3::up())
        //     * Quaternion::from_rad_axis(helper::angle_2_rad(z_angle), Vec3::right());
        // println!("norm of q {:?}, q: {:?} ", qq.norm(), qq);
        // let qqq = Quaternion::from_rad_axis(helper::angle_2_rad(y_angle), Vec3::up())
        //     * Quaternion::from_rad_axis(helper::angle_2_rad(x_angle), Vec3::right())
        //     * Quaternion::from_rad_axis(helper::angle_2_rad(z_angle), Vec3::forward());
        // println!("norm of q {:?}, q: {:?} ", qqq.norm(), qqq);

        println!("quat:{:?}", q.rotate(&v));
        println!("matrix from quat:{:?}", q.to_mat3() * v);
        println!("RotationMatrix:{:?}", m * v);

        // assert_approx_eq!(r[0], ZERO);
        // assert_approx_eq!(r[1], 0.7071067);
        // assert_approx_eq!(r[2], 0.7071067);
    }
}
