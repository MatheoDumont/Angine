use super::Real;
use num::traits::Float;

pub struct Matrix<Type: Float, const Row: usize, const Col: usize> {
    values: [[Type; Row]; Col],
}

impl<Type: Float, const Row: usize, const Col: usize> Matrix<Type, Row, Col> {
    pub fn zero() -> Matrix<Type, Row, Col> {
        Matrix {
            values: [[Type::zero(); Row]; Col],
        }
    }
}

type Mat3 = Matrix<Real, 3, 3>;
type Mat4 = Matrix<Real, 4, 4>;
