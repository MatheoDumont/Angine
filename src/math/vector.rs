use super::{Real, ONE, ZERO};
use std::cmp::{Eq, PartialEq};
use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

#[derive(Copy, Clone, Debug)]
pub struct Vector3 {
    pub data: [Real; 3],
}

impl Vector3 {
    pub fn new(v1: Real, v2: Real, v3: Real) -> Vector3 {
        Vector3 { data: [v1, v2, v3] }
    }

    pub fn from_array(data: [Real; 3]) -> Vector3 {
        Vector3 { data }
    }

    pub fn value(v: Real) -> Vector3 {
        Vector3 { data: [v; 3] }
    }

    pub fn zeros() -> Vector3 {
        Vector3::value(ZERO)
    }

    pub fn ones() -> Vector3 {
        Vector3::value(ONE)
    }

    pub fn origin() -> Vector3 {
        Vector3::zeros()
    }

    pub fn x(&self) -> Real {
        self.data[0]
    }

    pub fn y(&self) -> Real {
        self.data[1]
    }

    pub fn z(&self) -> Real {
        self.data[2]
    }
}

pub fn squared_magnitude(v: &Vector3) -> Real {
    v[0].powi(2) + v[1].powi(2) + v[2].powi(2)
}

pub fn magnitude(v: &Vector3) -> Real {
    squared_magnitude(v).sqrt()
}

pub fn normalize(v: &mut Vector3) {
    *v /= magnitude(v);
}

pub fn normalized(v: &Vector3) -> Vector3 {
    let l = magnitude(v);
    Vector3::from_array([v[0] / l, v[1] / l, v[2] / l])
}

pub fn dot(v1: &Vector3, v2: &Vector3) -> Real {
    v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
}

pub fn cross(v1: &Vector3, v2: &Vector3) -> Vector3 {
    Vector3::new(
        v1.y() * v2.z() - v1.z() * v2.y(),
        v1.z() * v2.x() - v1.x() * v2.z(),
        v1.x() * v2.y() - v1.y() * v2.x(),
    )
}

/**
 * Project A onto B.
 * A is the normal unit vector representing the plane onto B is projected.
 * A is normalized
 */
pub fn projection(A: &Vector3, B: &Vector3) -> Vector3 {
    A * dot(A, B)
}

/**
 * Returns the closest perpendicular part of B to A
 * A is normalized
 */
pub fn rejection(A: &Vector3, B: &Vector3) -> Vector3 {
    B - &projection(A, B)
}

/**
 * A is the axis of symmetry where the reflection occur
 * A is normalized
 */
pub fn reflection(A: &Vector3, B: &Vector3) -> Vector3 {
    let perp2 = rejection(A, B) * (2 as Real);
    B - &perp2
}

pub struct Directions;
impl Directions {
    pub fn up() -> Vector3 {
        Vector3::new(ZERO, ONE, ZERO)
    }
    pub fn down() -> Vector3 {
        Vector3::new(ZERO, -ONE, ZERO)
    }
    pub fn forward() -> Vector3 {
        Vector3::new(ZERO, ZERO, ONE)
    }
    pub fn backward() -> Vector3 {
        Vector3::new(ZERO, ZERO, -ONE)
    }
    pub fn left() -> Vector3 {
        Vector3::new(-ONE, ZERO, ZERO)
    }
    pub fn right() -> Vector3 {
        Vector3::new(ONE, ZERO, ZERO)
    }
}

impl Neg for Vector3 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Vector3 {
            data: [-self.data[0], -self.data[1], -self.data[2]],
        }
    }
}

impl Neg for &Vector3 {
    type Output = Vector3;

    fn neg(self) -> Self::Output {
        Vector3 {
            data: [-self.data[0], -self.data[1], -self.data[2]],
        }
    }
}

impl Add for Vector3 {
    type Output = Self;

    fn add(self, v: Self) -> Self::Output {
        Vector3 {
            data: [self[0] + v[0], self[1] + v[1], self[2] + v[2]],
        }
    }
}

impl Add for &Vector3 {
    type Output = Vector3;

    fn add(self, v: Self) -> Self::Output {
        Vector3 {
            data: [self[0] + v[0], self[1] + v[1], self[2] + v[2]],
        }
    }
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, v: Self) -> Self::Output {
        Vector3 {
            data: [self[0] - v[0], self[1] - v[1], self[2] - v[2]],
        }
    }
}

impl Sub for &Vector3 {
    type Output = Vector3;

    fn sub(self, v: Self) -> Self::Output {
        Vector3 {
            data: [self[0] - v[0], self[1] - v[1], self[2] - v[2]],
        }
    }
}

impl Div<Real> for Vector3 {
    type Output = Vector3;

    fn div(self, divisor: Real) -> Self::Output {
        Vector3 {
            data: [self[0] / divisor, self[1] / divisor, self[2] / divisor],
        }
    }
}

impl Div<Real> for &Vector3 {
    type Output = Vector3;

    fn div(self, divisor: Real) -> Self::Output {
        Vector3 {
            data: [self[0] / divisor, self[1] / divisor, self[2] / divisor],
        }
    }
}

impl Mul for Vector3 {
    type Output = Self;

    fn mul(self, v: Self) -> Self {
        Vector3 {
            data: [self[0] * v[0], self[1] * v[1], self[2] * v[2]],
        }
    }
}

impl Mul<Real> for Vector3 {
    type Output = Vector3;

    fn mul(self, rhs: Real) -> Self::Output {
        Vector3 {
            data: [self[0] * rhs, self[1] * rhs, self[2] * rhs],
        }
    }
}

impl Mul<Real> for &Vector3 {
    type Output = Vector3;

    fn mul(self, rhs: Real) -> Self::Output {
        Vector3 {
            data: [self[0] * rhs, self[1] * rhs, self[2] * rhs],
        }
    }
}

impl AddAssign for Vector3 {
    fn add_assign(&mut self, v: Self) {
        self[0] += v[0];
        self[1] += v[1];
        self[2] += v[2];
    }
}
impl SubAssign for Vector3 {
    fn sub_assign(&mut self, v: Self) {
        self[0] -= v[0];
        self[1] -= v[1];
        self[2] -= v[2];
    }
}

impl DivAssign<Real> for Vector3 {
    fn div_assign(&mut self, divisor: Real) {
        self[0] /= divisor;
        self[1] /= divisor;
        self[2] /= divisor;
    }
}

impl MulAssign<Real> for Vector3 {
    fn mul_assign(&mut self, m: Real) {
        self[0] *= m;
        self[1] *= m;
        self[2] *= m;
    }
}

impl Index<usize> for Vector3 {
    type Output = Real;
    fn index(&self, index: usize) -> &Self::Output {
        debug_assert!(index < 3);

        &self.data[index]
        // let ptr_x = std::ptr::addr_of!(self.x);

        // let r = unsafe { ptr_x.offset(index.try_into().unwrap()).as_ref().unwrap() };
        // r
    }
}

impl IndexMut<usize> for Vector3 {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        debug_assert!(index < 3);
        &mut self.data[index]
        // let ptr_x = std::ptr::addr_of_mut!(self.x); // sinon &mut self.x as *mut Real

        // let r = unsafe { ptr_x.offset(index.try_into().unwrap()).as_mut().unwrap() };
        // r
    }
}

impl PartialEq for Vector3 {
    fn eq(&self, other: &Self) -> bool {
        self.data[0] == other.data[0]
            && self.data[1] == other.data[1]
            && self.data[2] == other.data[2]
    }
}
impl Eq for Vector3 {}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn vector_div() {
        let v = Vector3::new(2 as Real, 4 as Real, 6 as Real);
        let vv = &v / 2 as Real;
        assert_eq!(vv.x(), 1 as Real);
        assert_eq!(vv.y(), 2 as Real);
        assert_eq!(vv.z(), 3 as Real);
    }

    #[test]
    fn vector_div_assign() {
        let mut v = Vector3::new(2f32, 4f32, 6f32);
        v /= 2f32;
        assert_eq!(v.x(), 1f32);
        assert_eq!(v.y(), 2f32);
        assert_eq!(v.z(), 3f32);
    }

    #[test]
    fn vector_normalize() {
        let r = 56f32.sqrt();
        let mut v = Vector3::new(2f32, 4f32, 6f32);
        normalize(&mut v);

        assert_eq!(v.x(), 2f32 / r);
        assert_eq!(v.y(), 4f32 / r);
        assert_eq!(v.z(), 6f32 / r);
    }

    #[test]
    fn vector_normalized() {
        let r = 56f32.sqrt();
        let v = Vector3::new(2f32, 4f32, 6f32);
        let vv = normalized(&v);

        assert_eq!(vv.x(), 2f32 / r);
        assert_eq!(vv.y(), 4f32 / r);
        assert_eq!(vv.z(), 6f32 / r);
    }

    #[test]
    fn vector_dot_product() {
        let v = Vector3::new(1 as Real, 2 as Real, 3 as Real);
        let vv = Vector3::new(0.5, 0.4, 0.6);

        assert_eq!(dot(&v, &vv), 0.5 + 0.8 + 1.8);
    }

    #[test]
    fn test_orientation_cross_product_vec() {
        // since we're using a left hand coordinate system
        {
            let r = cross(&Directions::up(), &Directions::forward());
            assert_eq!(r.x(), ONE);
            assert_eq!(r.y(), ZERO);
            assert_eq!(r.z(), ZERO);
        }

        {
            let r = cross(&Directions::forward(), &Directions::right());
            assert_eq!(r.x(), ZERO);
            assert_eq!(r.y(), ONE);
            assert_eq!(r.z(), ZERO);
        }

        {
            let r = cross(&Directions::right(), &Directions::up());
            assert_eq!(r.x(), ZERO);
            assert_eq!(r.y(), ZERO);
            assert_eq!(r.z(), ONE);
        }
    }
}
