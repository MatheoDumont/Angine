use super::{Shape, ShapeType};
use crate::math::{math_essentials::*, Mat3};

pub struct Sphere {
    pub radius: Real, // radius in ]0, +inf[ but no check
    pub position: P3,
}

impl Sphere {
    pub fn new(radius: Real) -> Self {
        Self {
            radius: radius,
            position: P3::origin(),
        }
    }

    pub fn closest_point(&self, p: &P3) -> P3 {
        let mut d = p - &self.position;
        normalize(&mut d);
        self.position + d * self.radius
    }
}

impl Shape for Sphere {
    fn compute_inertia_matrix(&self, mass: Real) -> Mat3 {
        // pour l'instant
        // panic!("inertia matrix for Sphere not implemented");

        

        Mat3::diag(Vec3::value(0.4 * mass * self.radius.powi(2)))
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Sphere
    }

    fn is_rigid_body(&self) -> bool {
        true
    }

    fn get_position(&self) -> &P3 {
        &self.position
    }
    // bidon
    fn get_orientation(&self) -> &Mat3 {
        panic!("get_orientation() on sphere not implemented !");
    }
    fn set_position(&mut self, p: P3) {
        self.position = p;
    }
    fn set_orientation(&mut self, o: Mat3) {}

    fn get_transform(&self) -> Transform {
        Transform::translation(self.position)
    }
    fn set_transform(&mut self, t: Transform) {
        self.position = t.translation;
    }
}
