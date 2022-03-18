use super::{Real, ONE, P3, ZERO};
use std::{
    convert::From,
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign},
};

#[derive(Copy, Clone, Debug)]

pub struct Vector<const SIZE: usize> {
    data: [Real; SIZE],
}

impl<const SIZE: usize> Neg for Vector<SIZE> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = -self.data[i];
        }
        new
    }
}

impl<const SIZE: usize> Neg for &Vector<SIZE> {
    type Output = Vector<SIZE>;

    fn neg(self) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = -self.data[i];
        }
        new
    }
}

impl<const SIZE: usize> Add for Vector<SIZE> {
    type Output = Self;

    fn add(self, v: Self) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = self.data[i] + v.data[i];
        }
        new
    }
}

impl<const SIZE: usize> Add for &Vector<SIZE> {
    type Output = Vector<SIZE>;

    fn add(self, v: Self) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = self.data[i] + v.data[i];
        }
        new
    }
}

impl<const SIZE: usize> Sub for Vector<SIZE> {
    type Output = Self;

    fn sub(self, v: Self) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = self.data[i] - v.data[i];
        }
        new
    }
}

impl<const SIZE: usize> Sub for &Vector<SIZE> {
    type Output = Vector<SIZE>;

    fn sub(self, v: Self) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = self.data[i] - v.data[i];
        }
        new
    }
}

impl<const SIZE: usize> AddAssign for Vector<SIZE> {
    fn add_assign(&mut self, v: Self) {
        for i in 0..SIZE {
            self.data[i] += v.data[i];
        }
    }
}
impl<const SIZE: usize> SubAssign for Vector<SIZE> {
    fn sub_assign(&mut self, v: Self) {
        for i in 0..SIZE {
            self.data[i] -= v.data[i];
        }
    }
}

impl<const SIZE: usize> Div<Real> for Vector<SIZE> {
    type Output = Vector<SIZE>;

    fn div(self, divisor: Real) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = self.data[i] / divisor;
        }
        new
    }
}

impl<const SIZE: usize> Div<Real> for &Vector<SIZE> {
    type Output = Vector<SIZE>;

    fn div(self, divisor: Real) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = self.data[i] / divisor;
        }
        new
    }
}

impl<const SIZE: usize> DivAssign<Real> for Vector<SIZE> {
    fn div_assign(&mut self, divisor: Real) {
        for i in 0..SIZE {
            self.data[i] /= divisor;
        }
    }
}

impl<const SIZE: usize> Mul<Real> for Vector<SIZE> {
    type Output = Vector<SIZE>;

    fn mul(self, m: Real) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = self.data[i] * m;
        }
        new
    }
}

impl<const SIZE: usize> Mul<Real> for &Vector<SIZE> {
    type Output = Vector<SIZE>;

    fn mul(self, m: Real) -> Self::Output {
        let mut new = Vector { data: [ZERO; SIZE] };
        for i in 0..SIZE {
            new.data[i] = self.data[i] * m;
        }
        new
    }
}

impl<const SIZE: usize> MulAssign<Real> for Vector<SIZE> {
    fn mul_assign(&mut self, m: Real) {
        for i in 0..SIZE {
            self.data[i] *= m;
        }
    }
}

impl<const SIZE: usize> Index<usize> for Vector<SIZE> {
    type Output = Real;
    fn index(&self, index: usize) -> &Self::Output {
        debug_assert!(index < SIZE);

        &self.data[index]
        // let ptr_x = std::ptr::addr_of!(self.x);

        // let r = unsafe { ptr_x.offset(index.try_into().unwrap()).as_ref().unwrap() };
        // r
    }
}

impl<const SIZE: usize> IndexMut<usize> for Vector<SIZE> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        debug_assert!(index < SIZE);
        &mut self.data[index]
        // let ptr_x = std::ptr::addr_of_mut!(self.x); // sinon &mut self.x as *mut Real

        // let r = unsafe { ptr_x.offset(index.try_into().unwrap()).as_mut().unwrap() };
        // r
    }
}
