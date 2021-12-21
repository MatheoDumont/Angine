use crate::math::{vector::Vector, Real};
use std::{convert::From, ops::Add, ops::AddAssign, ops::Sub};

pub struct Point {
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

impl Point {
    pub fn new(x: Real, y: Real, z: Real) -> Point {
        Point { x: x, y: y, z: z }
    }

    pub fn origin() -> Point {
        Point {
            x: 0 as Real,
            y: 0 as Real,
            z: 0 as Real,
        }
    }
}

impl From<Vector> for Point {
    fn from(p: Vector) -> Self {
        Point::new(p.x, p.y, p.z)
    }
}

impl Sub for Point {
    type Output = Vector;

    fn sub(self, sub: Self) -> Self::Output {
        Vector::new(self.x - sub.x, self.y - sub.y, self.z - sub.z)
    }
}

impl Sub for &Point {
    type Output = Vector;

    fn sub(self, sub: Self) -> Self::Output {
        Vector::new(self.x - sub.x, self.y - sub.y, self.z - sub.z)
    }
}

impl Add<Vector> for &Point {
    type Output = Point;
    fn add(self, a: Vector) -> Self::Output {
        Point::new(self.x + a.x, self.y + a.y, self.z + a.z)
    }
}
