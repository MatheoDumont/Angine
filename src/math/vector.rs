use crate::math::{point::Point, Real};
use std::{
    convert::From,
    ops::{Div, DivAssign, Mul, MulAssign},
};

pub struct Vector {
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

impl From<Point> for Vector {
    fn from(p: Point) -> Self {
        Vector::new(p.x, p.y, p.z)
    }
}

impl Div<Real> for Vector {
    type Output = Vector;

    fn div(self, divisor: Real) -> Self::Output {
        Vector::new(self.x / divisor, self.y / divisor, self.z / divisor)
    }
}

impl Div<Real> for &Vector {
    type Output = Vector;

    fn div(self, divisor: Real) -> Self::Output {
        Vector::new(self.x / divisor, self.y / divisor, self.z / divisor)
    }
}

impl DivAssign<Real> for Vector {
    fn div_assign(&mut self, divisor: Real) {
        self.x /= divisor;
        self.y /= divisor;
        self.z /= divisor;
    }
}

impl Mul<Real> for Vector {
    type Output = Vector;

    fn mul(self, m: Real) -> Self::Output {
        Vector {
            x: self.x * m,
            y: self.y * m,
            z: self.z * m,
        }
    }
}

impl Mul<Real> for &Vector {
    type Output = Vector;

    fn mul(self, m: Real) -> Self::Output {
        Vector {
            x: self.x * m,
            y: self.y * m,
            z: self.z * m,
        }
    }
}

impl MulAssign<Real> for Vector {
    fn mul_assign(&mut self, m: Real) {
        self.x *= m;
        self.y *= m;
        self.z *= m;
    }
}

impl Vector {
    pub fn new(x: Real, y: Real, z: Real) -> Vector {
        Vector { x: x, y: y, z: z }
    }
    pub fn zeros() -> Vector {
        Vector::new(0 as Real, 0 as Real, 0 as Real)
    }

    pub fn norm(&self) -> Real {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    pub fn normalize(&mut self) {
        let n = self.norm();
        self.div_assign(n);
    }

    pub fn normalized(&self) -> Vector {
        let n = self.norm();
        self.div(n)
    }
}

#[cfg(test)]
mod tests {
    use super::Vector;
    #[test]
    fn vector_div() {
        let v = Vector::new(2f32, 4f32, 6f32);
        let vv = v / 2f32;
        assert_eq!(vv.x, 1f32);
        assert_eq!(vv.y, 2f32);
        assert_eq!(vv.z, 3f32);
    }

    #[test]
    fn vector_div_assign() {
        let mut v = Vector::new(2f32, 4f32, 6f32);
        v /= 2f32;
        assert_eq!(v.x, 1f32);
        assert_eq!(v.y, 2f32);
        assert_eq!(v.z, 3f32);
    }

    #[test]
    fn vector_normalize() {
        let r = 56f32.sqrt();
        let mut v = Vector::new(2f32, 4f32, 6f32);
        v.normalize();

        assert_eq!(v.x, 2f32 / r);
        assert_eq!(v.y, 4f32 / r);
        assert_eq!(v.z, 6f32 / r);
    }

    #[test]
    fn vector_normalized() {
        let r = 56f32.sqrt();
        let v = Vector::new(2f32, 4f32, 6f32);
        let vv = v.normalized();

        assert_eq!(v.x, 2f32);
        assert_eq!(v.y, 4f32);
        assert_eq!(v.z, 6f32);

        assert_eq!(vv.x, 2f32 / r);
        assert_eq!(vv.y, 4f32 / r);
        assert_eq!(vv.z, 6f32 / r);
    }
}
