use crate::collision_detection::shapes::{Sphere, OBB};

pub fn obb_sphere(obb: &OBB, sphere: &Sphere) -> bool {
    (sphere.position - obb.closest_point(&sphere.position)).norm_squared()
        <= (sphere.radius * sphere.radius)
}

#[cfg(test)]
mod tests {
    use super::obb_sphere;
    use crate::collision_detection::shapes::{Sphere, OBB};
    use crate::math::{Real, Rotation, Transform, Vec3, ONE, P3, ZERO};
    #[test]
    fn obb_sphere_intersection() {
        // no intersection
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let sphere = Sphere::new(ONE, P3::new(3 as Real, ZERO, ZERO));

            assert_eq!(obb_sphere(&obb, &sphere), false);
        }
        // intersection contour
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let sphere = Sphere::new(ONE, P3::new(2 as Real, ZERO, ZERO));

            assert_eq!(obb_sphere(&obb, &sphere), true);
        }

        // intersection
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let sphere = Sphere::new(ONE, P3::new(1.5 as Real, ZERO, ZERO));

            assert_eq!(obb_sphere(&obb, &sphere), true);
        }
        // rotated
        {
            let obb = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4)),
            );
            let sphere = Sphere::new(ONE, P3::new(1.5 as Real, ZERO, ZERO));

            assert_eq!(obb_sphere(&obb, &sphere), true);
        }
    }
}
