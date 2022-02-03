use super::{Real, Vec3, ZERO};
use std::{convert::From, ops::Add, ops::Index, ops::IndexMut, ops::Sub, ops::AddAssign, ops::SubAssign};

#[derive(Copy, Clone, Debug)]
pub struct P3 {
    pub values: [Real; 3],
}

impl P3 {
    pub fn new(x: Real, y: Real, z: Real) -> P3 {
        P3 { values: [x, y, z] }
    }

    pub fn origin() -> P3 {
        P3 { values: [ZERO; 3] }
    }
}

impl Index<usize> for P3 {
    type Output = Real;
    fn index(&self, index: usize) -> &Self::Output {
        &self.values[index]
    }
}
impl IndexMut<usize> for P3 {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.values[index]
    }
}

impl From<&Vec3> for P3 {
    fn from(p: &Vec3) -> Self {
        P3::new(p.x, p.y, p.z)
    }
}

impl From<Vec3> for P3 {
    fn from(p: Vec3) -> Self {
        P3::new(p.x, p.y, p.z)
    }
}

// impl Sub for P3 {
//     type Output = Vec3;

//     fn sub(self, sub: Self) -> Self::Output {
//         Vec3::new(self.x - sub.x, self.y - sub.y, self.z - sub.z)
//     }
// }

impl Sub for P3 {
    type Output = Vec3;

    fn sub(self, sub: Self) -> Self::Output {
        Vec3::new(
            self.values[0] - sub.values[0],
            self.values[1] - sub.values[1],
            self.values[2] - sub.values[2],
        )
    }
}

impl Sub for &P3 {
    type Output = Vec3;

    fn sub(self, sub: Self) -> Self::Output {
        Vec3::new(
            self.values[0] - sub.values[0],
            self.values[1] - sub.values[1],
            self.values[2] - sub.values[2],
        )
    }
}

impl Sub<Vec3> for P3 {
    type Output = P3;

    fn sub(self, v: Vec3) -> Self::Output {
        P3::new(
            self.values[0] - v.x,
            self.values[1] - v.y,
            self.values[2] - v.z,
        )
    }
}

impl Sub<&Vec3> for &P3 {
    type Output = P3;

    fn sub(self, v: &Vec3) -> Self::Output {
        P3::new(
            self.values[0] - v.x,
            self.values[1] - v.y,
            self.values[2] - v.z,
        )
    }
}

impl Add<Vec3> for P3 {
    type Output = P3;
    fn add(self, a: Vec3) -> Self::Output {
        P3::new(
            self.values[0] + a.x,
            self.values[1] + a.y,
            self.values[2] + a.z,
        )
    }
}

impl Add<&Vec3> for P3 {
    type Output = P3;
    fn add(self, a: &Vec3) -> Self::Output {
        P3::new(
            self.values[0] + a.x,
            self.values[1] + a.y,
            self.values[2] + a.z,
        )
    }
}

impl Add<Vec3> for &P3 {
    type Output = P3;
    fn add(self, a: Vec3) -> Self::Output {
        P3::new(
            self.values[0] + a.x,
            self.values[1] + a.y,
            self.values[2] + a.z,
        )
    }
}

impl Add<&Vec3> for &P3 {
    type Output = P3;
    fn add(self, a: &Vec3) -> Self::Output {
        P3::new(
            self.values[0] + a.x,
            self.values[1] + a.y,
            self.values[2] + a.z,
        )
    }
}

impl AddAssign<Vec3> for P3 {
    fn add_assign(&mut self, v: Vec3) {
        self[0] += v.x;
        self[1] += v.y;
        self[2] += v.z;

    }
}

impl SubAssign<Vec3> for P3 {
    fn sub_assign(&mut self, v: Vec3) {
        self[0] -= v.x;
        self[1] -= v.y;
        self[2] -= v.z;

    }
}
