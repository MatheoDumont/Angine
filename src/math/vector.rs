use super::{Real, P3};
use std::{
    convert::From,
    ops::{Div, DivAssign, Mul, MulAssign},
};

pub struct Vec3 {
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

impl From<&P3> for Vec3 {
    fn from(p: &P3) -> Self {
        Vec3::new(p.x, p.y, p.z)
    }
}

// impl Div<Real> for Vec3 {
//     type Output = Vec3;

//     fn div(self, divisor: Real) -> Self::Output {
//         Vec3::new(self.x / divisor, self.y / divisor, self.z / divisor)
//     }
// }

impl Div<Real> for &Vec3 {
    type Output = Vec3;

    fn div(self, divisor: Real) -> Self::Output {
        Vec3::new(self.x / divisor, self.y / divisor, self.z / divisor)
    }
}

impl DivAssign<Real> for Vec3 {
    fn div_assign(&mut self, divisor: Real) {
        self.x /= divisor;
        self.y /= divisor;
        self.z /= divisor;
    }
}

impl Mul for &Vec3 {
    type Output = Real;

    fn mul(self, m: Self) -> Self::Output {
        self.dot(m)
    }
}

// impl Mul<Real> for Vec3 {
//     type Output = Vec3;

//     fn mul(self, m: Real) -> Self::Output {
//         Vec3 {
//             x: self.x * m,
//             y: self.y * m,
//             z: self.z * m,
//         }
//     }
// }

impl Mul<Real> for &Vec3 {
    type Output = Vec3;

    fn mul(self, m: Real) -> Self::Output {
        Vec3 {
            x: self.x * m,
            y: self.y * m,
            z: self.z * m,
        }
    }
}

impl MulAssign<Real> for Vec3 {
    fn mul_assign(&mut self, m: Real) {
        self.x *= m;
        self.y *= m;
        self.z *= m;
    }
}

impl Vec3 {
    pub fn new(x: Real, y: Real, z: Real) -> Vec3 {
        Vec3 { x: x, y: y, z: z }
    }
    pub fn zeros() -> Vec3 {
        Vec3::new(0 as Real, 0 as Real, 0 as Real)
    }

    pub fn norm(&self) -> Real {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    pub fn normalize(&mut self) {
        let n = self.norm();
        self.div_assign(n);
    }

    pub fn normalized(&self) -> Vec3 {
        let n = self.norm();
        self.div(n)
    }

    pub fn dot(&self, o: &Vec3) -> Real {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn vector_div() {
        let v = Vec3::new(2 as Real, 4 as Real, 6 as Real);
        let vv = &v / 2 as Real;
        assert_eq!(vv.x, 1 as Real);
        assert_eq!(vv.y, 2 as Real);
        assert_eq!(vv.z, 3 as Real);
    }

    #[test]
    fn vector_div_assign() {
        let mut v = Vec3::new(2f32, 4f32, 6f32);
        v /= 2f32;
        assert_eq!(v.x, 1f32);
        assert_eq!(v.y, 2f32);
        assert_eq!(v.z, 3f32);
    }

    #[test]
    fn vector_normalize() {
        let r = 56f32.sqrt();
        let mut v = Vec3::new(2f32, 4f32, 6f32);
        v.normalize();

        assert_eq!(v.x, 2f32 / r);
        assert_eq!(v.y, 4f32 / r);
        assert_eq!(v.z, 6f32 / r);
    }

    #[test]
    fn vector_normalized() {
        let r = 56f32.sqrt();
        let v = Vec3::new(2f32, 4f32, 6f32);
        let vv = v.normalized();

        assert_eq!(v.x, 2f32);
        assert_eq!(v.y, 4f32);
        assert_eq!(v.z, 6f32);

        assert_eq!(vv.x, 2f32 / r);
        assert_eq!(vv.y, 4f32 / r);
        assert_eq!(vv.z, 6f32 / r);
    }

    #[test]
    fn vector_dot_product() {
        let v = Vec3::new(1 as Real, 2 as Real, 3 as Real);
        let vv = Vec3::new(0.5, 0.4, 0.6);

        assert_eq!(&v * &vv, 0.5 + 0.8 + 1.8);
    }
}
