use super::{Real, ONE, P3, ZERO};
use std::{
    convert::From,
    ops::{Add, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub},
};

#[derive(Copy, Clone, Debug)]
pub struct Vec3 {
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

impl From<&P3> for Vec3 {
    fn from(p: &P3) -> Self {
        Vec3::new(p[0], p[1], p[2])
    }
}
impl From<P3> for Vec3 {
    fn from(p: P3) -> Self {
        Vec3::new(p[0], p[1], p[2])
    }
}

impl Neg for Vec3 {
    type Output = Vec3;

    fn neg(self) -> Self::Output {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}


impl Neg for &Vec3 {
    type Output = Vec3;

    fn neg(self) -> Self::Output {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

impl Add for Vec3 {
    type Output = Vec3;

    fn add(self, v: Self) -> Self::Output {
        Vec3::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}

impl Add for &Vec3 {
    type Output = Vec3;

    fn add(self, v: Self) -> Self::Output {
        Vec3::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;

    fn sub(self, v: Self) -> Self::Output {
        Vec3::new(self.x - v.x, self.y - v.y, self.z - v.z)
    }
}

impl Sub for &Vec3 {
    type Output = Vec3;

    fn sub(self, v: Self) -> Self::Output {
        Vec3::new(self.x - v.x, self.y - v.y, self.z - v.z)
    }
}

impl Div<Real> for Vec3 {
    type Output = Vec3;

    fn div(self, divisor: Real) -> Self::Output {
        Vec3::new(self.x / divisor, self.y / divisor, self.z / divisor)
    }
}

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

impl Mul for Vec3 {
    type Output = Real;

    fn mul(self, m: Self) -> Self::Output {
        self.dot(&m)
    }
}

impl Mul for &Vec3 {
    type Output = Real;

    fn mul(self, m: Self) -> Self::Output {
        self.dot(m)
    }
}

impl Mul<Real> for Vec3 {
    type Output = Vec3;

    fn mul(self, m: Real) -> Self::Output {
        Vec3 {
            x: self.x * m,
            y: self.y * m,
            z: self.z * m,
        }
    }
}

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

impl Index<usize> for Vec3 {
    type Output = Real;
    fn index(&self, index: usize) -> &Self::Output {
        debug_assert!(index < 3);
        let ptr_x = std::ptr::addr_of!(self.x);

        let r = unsafe { ptr_x.offset(index.try_into().unwrap()).as_ref().unwrap() };
        r
    }
}

impl IndexMut<usize> for Vec3 {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        debug_assert!(index < 3);
        let ptr_x = std::ptr::addr_of_mut!(self.x); // sinon &mut self.x as *mut Real

        let r = unsafe { ptr_x.offset(index.try_into().unwrap()).as_mut().unwrap() };
        r
    }
}

impl Vec3 {
    pub fn new(x: Real, y: Real, z: Real) -> Vec3 {
        Vec3 { x: x, y: y, z: z }
    }
    pub fn zero() -> Vec3 {
        Vec3::new(ZERO, ZERO, ZERO)
    }
    pub fn ones() -> Vec3 {
        Vec3::new(ONE, ONE, ONE)
    }

    pub fn norm_squared(&self) -> Real {
        self.x.powi(2) + self.y.powi(2) + self.z.powi(2)
    }

    pub fn norm(&self) -> Real {
        self.norm_squared().sqrt()
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
    pub fn dot_point(&self, p: &P3) -> Real {
        self.x * p[0] + self.y * p[1] + self.z * p[2]
    }

    pub fn cross(&self, o: &Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    /**
     * This fn return the projection of projected onto self,
     * this is equivalent to :
     *      self * self.dot(projected)
     * if self is normalized i.e self.norm() == 1
     *
     * for better efficiency, use project_normalized
     */
    pub fn project(&self, projected: &Vec3) -> Vec3 {
        self * (self.dot(projected) / self.norm_squared())
    }

    pub fn project_normalized(&self, projected: &Vec3) -> Vec3 {
        self * self.dot(projected)
    }

    /**
     * returns the perpendicular part of v to self
     */
    pub fn perpendicular(&self, v: &Vec3) -> Vec3 {
        v - &self.project(v)
    }

    pub fn perpendicular_normalized(&self, v: &Vec3) -> Vec3 {
        v - &self.project_normalized(v)
    }

    /**
     * Self is the axis of symmetry where the reflection occur
     */
    pub fn reflection(&self, d: &Vec3) -> Vec3 {
        let perp2 = &self.perpendicular(d) * (2 as Real);
        d - &perp2
    }

    pub fn reflection_normalized(&self, d: &Vec3) -> Vec3 {
        let perp2 = &self.perpendicular_normalized(d) * (2 as Real);
        d - &perp2
    }

    pub fn up() -> Vec3 {
        Vec3 {
            x: ZERO,
            y: ONE,
            z: ZERO,
        }
    }
    pub fn down() -> Vec3 {
        Vec3 {
            x: ZERO,
            y: -ONE,
            z: ZERO,
        }
    }
    pub fn forward() -> Vec3 {
        Vec3 {
            x: ZERO,
            y: ZERO,
            z: ONE,
        }
    }
    pub fn backward() -> Vec3 {
        Vec3 {
            x: ZERO,
            y: ZERO,
            z: -ONE,
        }
    }
    pub fn left() -> Vec3 {
        Vec3 {
            x: -ONE,
            y: ZERO,
            z: ZERO,
        }
    }
    pub fn right() -> Vec3 {
        Vec3 {
            x: ONE,
            y: ZERO,
            z: ZERO,
        }
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

    #[test]
    fn vector_index() {
        let mut v = Vec3::up();
        assert_eq!(v[0], ZERO);
        assert_eq!(v[1], ONE);
        assert_eq!(v[2], ZERO);

        v.x = 0.5;
        assert_eq!(v[0], 0.5);

        v[0] *= 2.0;
        assert_eq!(v[0], ONE);
    }
}
