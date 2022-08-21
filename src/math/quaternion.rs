use super::{helper, Directions, Mat3, Real, Vec3, ONE, TWO, ZERO};
use std::ops::{Add, Div, Mul, MulAssign};

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
        Quaternion::from_euler_rads(
            helper::angle_2_rad(x),
            helper::angle_2_rad(y),
            helper::angle_2_rad(z),
        )
    }

    /**
     * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
     */
    pub fn from_euler_rads(x_rad: Real, y_rad: Real, z_rad: Real) -> Quaternion {
        // Quaternion::from_rad_axis(y_rad, Directions::up())
        //     * Quaternion::from_rad_axis(x_rad, Directions::right())
        //     * Quaternion::from_rad_axis(z_rad, Directions::forward())
        // moins couteux et équivalent a la formule au dessus
        let rad1 = x_rad * 0.5;
        let rad2 = y_rad * 0.5;
        let rad3 = z_rad * 0.5;

        let c1 = rad1.cos();
        let s1 = rad1.sin();
        let c2 = rad2.cos();
        let s2 = rad2.sin();
        let c3 = rad3.cos();
        let s3 = rad3.sin();

        Quaternion {
            w: c1 * c3 * c2 + s2 * s1 * s3,
            x: c2 * s1 * c3 + s2 * c1 * s3,
            y: -c2 * s1 * s3 + s2 * c1 * c3,
            z: c2 * c1 * s3 - s2 * s1 * c3,
        }
    }

    /**
     * source pour la méthode utilisée: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
     * 16 multiplications + 15 addition/soustractions = 31 opérations
     * retourne la matrice homogène du quaternion associé
     */
    pub fn to_mat3(&self) -> Mat3 {
        // https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        // 18 multiplications, 12 addition/soustractions = 30 opérations
        // let xx2 = TWO * self.x * self.x;
        // let yy2 = TWO * self.y * self.y;
        // let zz2 = TWO * self.z * self.z;
        // let xy2 = TWO * self.x * self.y;
        // let zw2 = TWO * self.z * self.w;
        // let xz2 = TWO * self.x * self.z;
        // let yw2 = TWO * self.y * self.w;
        // let yz2 = TWO * self.y * self.z;
        // let xw2 = TWO * self.x * self.w;

        // Mat3::from_array([
        //     [ONE - yy2 - zz2, xy2 - zw2, xz2 + yw2],
        //     [xy2 + zw2, ONE - xx2 - zz2, yz2 - xw2],
        //     [xz2 - yw2, yz2 + xw2, ONE - xx2 - yy2],
        // ])

        // 16 *, 15 +/- = 31
        let ww = self.w * self.w;
        let xx = self.x * self.x;
        let yy = self.y * self.y;
        let zz = self.z * self.z;
        let xy = self.x * self.y;
        let wz = self.w * self.z;
        let wy = self.w * self.y;
        let xz = self.x * self.z;
        let yz = self.y * self.z;
        let wx = self.w * self.x;

        Mat3::from_array([
            [ww + xx - yy - zz, TWO * (xy - wz), TWO * (wy + xz)],
            [TWO * (xy + wz), ww - xx + yy - zz, TWO * (yz - wx)],
            [TWO * (xz - wy), TWO * (wx + yz), ww - xx - yy + zz],
        ])
    }

    /**
     * sources : https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
     *           https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/paul.htm
     */
    pub fn from_mat3(m: &Mat3) -> Quaternion {
        let w = helper::max(ZERO, ONE + m[0][0] + m[1][1] + m[2][2]).sqrt() / TWO;
        let x = helper::max(ZERO, ONE + m[0][0] - m[1][1] - m[2][2]).sqrt() / TWO;
        let y = helper::max(ZERO, ONE - m[0][0] + m[1][1] - m[2][2]).sqrt() / TWO;
        let z = helper::max(ZERO, ONE - m[0][0] - m[1][1] + m[2][2]).sqrt() / TWO;

        // let copysign = |a: Real, b: Real| {
        //     if b > ZERO {
        //         a.abs()
        //     } else if b < ZERO {
        //         -a.abs()
        //     } else {
        //         ZERO
        //     }
        // };

        Quaternion {
            w,
            x: x.copysign(m[2][1] - m[1][2]),
            y: y.copysign(m[0][2] - m[2][0]),
            z: z.copysign(m[1][0] - m[0][1]),
        }
    }

    pub fn conjugate(&self) -> Quaternion {
        Quaternion {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn squared_magnitude(&self) -> Real {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn magnitude(&self) -> Real {
        self.squared_magnitude().sqrt()
    }

    pub fn inverse(&self) -> Quaternion {
        self.conjugate() / self.squared_magnitude()
    }

    /**
     * 56 opérations pour rotate un vec
     */
    pub fn rotate(&self, v: &Vec3) -> Vec3 {
        let q = self * &Quaternion::from_vec(v) * self.conjugate();
        Quaternion::to_vec(&q)
    }

    pub fn quaternion_derivative(&self, quat_from_vec: Quaternion) -> Quaternion {
        self + &(&quat_from_vec * self * 0.5)
    }
}

impl Add for Quaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Quaternion {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Add for &Quaternion {
    type Output = Quaternion;

    fn add(self, rhs: Self) -> Self::Output {
        Quaternion {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
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

impl MulAssign<&Quaternion> for Quaternion {
    fn mul_assign(&mut self, rhs: &Quaternion) {
        self.w = self.w * rhs.w - (self.x * rhs.x + self.y * rhs.y + self.z * rhs.z);
        self.x = (self.w * rhs.x + self.x * rhs.w) + (self.y * rhs.z - rhs.y * self.z);
        self.y = (self.w * rhs.y + self.y * rhs.w) + (self.z * rhs.x - rhs.z * self.x);
        self.z = (self.w * rhs.z + self.z * rhs.w) + (self.x * rhs.y - rhs.x * self.y);
    }
}

impl Mul<Real> for Quaternion {
    type Output = Quaternion;
    fn mul(self, rhs: Real) -> Self::Output {
        Quaternion {
            w: self.w * rhs,
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
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
    use crate::math::math_essentials::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_axis_vs_angle() {
        let v = Directions::up();
        let q = Quaternion::from_euler_rads(
            helper::angle_2_rad(90.0),
            helper::angle_2_rad(ZERO),
            helper::angle_2_rad(ZERO),
        );
        let qq = Quaternion::from_rad_axis(helper::angle_2_rad(90.0), Directions::right());

        let vv = q.rotate(&v);
        let vvv = qq.rotate(&v);
        assert_eq!(
            helper::round_n_decimal_vector(&vv, 6),
            helper::round_n_decimal_vector(&vvv, 6)
        );

        println!(
            "{:?}",
            Quaternion::from_euler_rads(
                helper::angle_2_rad(90.0),
                helper::angle_2_rad(ZERO),
                helper::angle_2_rad(ZERO),
            )
            .to_mat3(),
        );
    }

    #[test]
    fn test_to_mat() {
        let q = Quaternion::from_euler_rads(
            helper::angle_2_rad(90.0),
            helper::angle_2_rad(ZERO),
            helper::angle_2_rad(ZERO),
        );
        let v = Directions::up();
        let vv = q.rotate(&v);
        let m = q.to_mat3();
        let vvv = m * v;

        assert_eq!(
            helper::round_n_decimal_vector(&vv, 6),
            helper::round_n_decimal_vector(&vvv, 6)
        );

        let x_angle = -30.0;
        let y_angle = 35.0;
        let z_angle = 45.0;
        let v = Vector3::new(0.5, 0.6, 0.7);

        // z then y
        let q = Quaternion::from_euler_angles(x_angle, y_angle, z_angle);
        let m = q.to_mat3();
        let vv = q.rotate(&v);
        let vvv = m * v;

        assert_eq!(
            helper::round_n_decimal_vector(&vv, 6),
            helper::round_n_decimal_vector(&vvv, 6)
        );
    }
    #[test]
    fn quaternion_rotation() {
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
    fn quat_composed_rotation_order_to_unity() {
        let x_angle = -30.0;
        let y_angle = 35.0;
        let z_angle = 45.0;

        // z then y
        let q = Quaternion::from_euler_angles(x_angle, y_angle, z_angle);
        // quaternion de Unity avec les memes angles x y z
        // let q = Quaternion {
        //     w: 0.8213125,
        //     x: -0.1168965,
        //     y: 0.3628112,
        //     z: 0.4244396,
        // };

        // println!("magnitude of q {:?}, q: {:?} ", q.magnitude(), q);

        assert_approx_eq!(0.8213125, q.w);
        assert_approx_eq!(-0.1168965, q.x);
        assert_approx_eq!(0.3628112, q.y);
        assert_approx_eq!(0.4244396, q.z);
    }
}
