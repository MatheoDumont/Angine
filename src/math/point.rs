use super::{Real, Vec3};
use std::{convert::From, ops::Add, ops::AddAssign, ops::Sub};

#[derive(Copy, Clone)]
pub struct P3 {
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

impl P3 {
    pub fn new(x: Real, y: Real, z: Real) -> P3 {
        P3 { x: x, y: y, z: z }
    }

    pub fn origin() -> P3 {
        P3 {
            x: 0 as Real,
            y: 0 as Real,
            z: 0 as Real,
        }
    }

    pub fn value(v: Real) -> P3 {
        P3 { x: v, y: v, z: v }
    }
}

impl From<&Vec3> for P3 {
    fn from(p: &Vec3) -> Self {
        P3::new(p.x, p.y, p.z)
    }
}

// impl Sub for P3 {
//     type Output = Vec3;

//     fn sub(self, sub: Self) -> Self::Output {
//         Vec3::new(self.x - sub.x, self.y - sub.y, self.z - sub.z)
//     }
// }

impl Sub for &P3 {
    type Output = Vec3;

    fn sub(self, sub: Self) -> Self::Output {
        Vec3::new(self.x - sub.x, self.y - sub.y, self.z - sub.z)
    }
}

impl Add<&Vec3> for P3 {
    type Output = P3;
    fn add(self, a: &Vec3) -> Self::Output {
        P3::new(self.x + a.x, self.y + a.y, self.z + a.z)
    }
}

impl Add<Vec3> for &P3 {
    type Output = P3;
    fn add(self, a: Vec3) -> Self::Output {
        P3::new(self.x + a.x, self.y + a.y, self.z + a.z)
    }
}

impl Add<&Vec3> for &P3 {
    type Output = P3;
    fn add(self, a: &Vec3) -> Self::Output {
        P3::new(self.x + a.x, self.y + a.y, self.z + a.z)
    }
}
