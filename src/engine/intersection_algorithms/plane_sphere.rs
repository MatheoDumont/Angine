use crate::engine::shapes::{Plane, Sphere};
use crate::math::math_essentials::*;

pub fn plane_sphere(plane: &Plane, sphere: &Sphere) -> bool {
    if plane.signed_distance(&sphere.position).abs() <= sphere.radius {
        true
    } else {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::plane_sphere;
    use crate::engine::shapes::{Plane, Sphere};
    use crate::math::math_essentials::*;
    #[test]
    fn plane_sphere_intersection() {
        // no intersection
        {
            let s = Sphere::new(ONE, P3::new(ZERO, 2 as Real, ZERO));
            let pl = Plane::new(Directions::up(), P3::origin());

            assert_eq!(plane_sphere(&pl, &s), false);
        }
        // intersection
        {
            let s = Sphere::new(ONE, P3::new(ZERO, ONE, ZERO));
            let pl = Plane::new(Directions::up(), P3::origin());

            assert_eq!(plane_sphere(&pl, &s), true);
        }

        {
            let s = Sphere::new(ONE, P3::new(ZERO, -ONE, ZERO));
            let pl = Plane::new(Directions::up(), P3::origin());

            assert_eq!(plane_sphere(&pl, &s), true);
        }

        {
            let s = Sphere::new(ONE, P3::new(ZERO, ONE, ZERO));
            let pl = Plane::new(Vec3::new(ONE, ONE, ZERO), P3::origin());

            assert_eq!(plane_sphere(&pl, &s), true);
        }
    }
}
