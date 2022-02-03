use super::{Shape, ShapeType};
use crate::math::{Real, Transform, Vec3, P3};

pub struct Plane {
    normal: Vec3,
    distance_from_origin: Real,
    transform: Transform,
}

impl Plane {
    pub fn new(normal: Vec3, position: P3) -> Plane {
        let v = Vec3::from(position);
        Plane {
            normal: normal,
            distance_from_origin: v.norm(),
            transform: Transform::translation(v),
        }
    }

    pub fn distance(&self, p: &P3) -> Real {
        &Vec3::from(p).dot(&self.normal) - self.distance_from_origin
    }
}

impl Shape for Plane {
    fn inertia_matrix(&self) -> Vec3 {
        // pour l'instant
        panic!("inertia matrix for Plane not implemented");
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Plane
    }
}

#[cfg(test)]
mod tests {
    use super::Plane;
    use crate::math::{Rotation, Vec3, ONE, P3, ZERO};
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn plane_distance() {
        {
            let plan = Plane::new(Vec3::up(), P3::origin());
            let p = P3::from(Vec3::up());

            assert_approx_eq!(plan.distance(&p), ONE, 1.0e-6);
        }

        {
            let plan = Plane::new(Vec3::up(), P3::origin());
            let p = P3::new(1.5, 1.0, 1.0);

            assert_approx_eq!(plan.distance(&p), Vec3::from(&p).dot(&plan.normal), 1.0e-6);
        }

        {
            let plan = Plane::new(
                Rotation::Y(std::f32::consts::FRAC_PI_4) * Vec3::right(),
                // P3::new(ZERO, ZERO, ONE),
                P3::origin(),
            );
            let p = P3::new(ZERO, ZERO, -ONE);
            println!("{:?}", plan.normal);
            assert_approx_eq!(plan.distance(&p), Vec3::from(&p).dot(&plan.normal), 1.0e-6);
        }
    }
}
