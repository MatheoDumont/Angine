use crate::engine::shapes::{Plane, OBB};
use crate::math::math_essentials::*;

pub fn obb_plane(obb: &OBB, plane: &Plane) -> bool {
    let rotation = &obb.transform.rotation;

    // distance of the closest point from the obb to the plane
    let closest_distance = obb.half_side.x() * dot(&plane.normal, &rotation.row(0)).abs()
        + obb.half_side.y() * dot(&plane.normal, &rotation.row(1)).abs()
        + obb.half_side.z() * dot(&plane.normal, &rotation.row(2)).abs();

    plane.signed_distance(&obb.transform.translation).abs() <= closest_distance
}

#[cfg(test)]
mod tests {
    use super::obb_plane;
    use crate::engine::shapes::{Plane, OBB};
    use crate::math::math_essentials::*;
    #[test]
    fn obb_plane_intersection() {
        // no intersection
        {
            let obb = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::new(ZERO, 2 as Real, ZERO)),
            );
            let plane = Plane::new(Directions::up(), P3::origin());
            assert_eq!(obb_plane(&obb, &plane), false);
        }
        // rotated obb no intersection
        {
            let obb = OBB::new(
                Vec3::new(0.6, 0.6, 0.6),
                Transform::new(
                    Vec3::zeros(),
                    Rotation::Z(std::f32::consts::FRAC_PI_4),
                    Vec3::new(ZERO, 1.5, ZERO),
                ),
            );
            let plane = Plane::new(
                Rotation::Z(std::f32::consts::FRAC_PI_4) * Directions::up(),
                P3::new(ZERO, ZERO, ZERO),
            );
            assert_eq!(obb_plane(&obb, &plane), false);
        }

        // rotated obb no intersection moved along z
        {
            let obb = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::new(
                    Vec3::zeros(),
                    Rotation::Z(std::f32::consts::FRAC_PI_4),
                    Vec3::new(ZERO, 1.5, 5 as Real),
                ),
            );
            let plane = Plane::new(Directions::up(), P3::new(ZERO, ZERO, ZERO));
            assert_eq!(obb_plane(&obb, &plane), false);
        }

        // intersection
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let plane = Plane::new(Directions::up(), P3::new(ZERO, -ONE, ZERO));
            assert_eq!(obb_plane(&obb, &plane), true);
        }
        // origin above, pt below plane
        {
            let obb = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::new(
                    Vec3::ones(),
                    Rotation::Z(std::f32::consts::FRAC_PI_4),
                    Vec3::new(ZERO, ONE, ZERO),
                ),
            );
            let plane = Plane::new(Directions::up(), P3::new(ZERO, ZERO, ZERO));
            assert_eq!(obb_plane(&obb, &plane), true);
        }
        // intersection on contour
        {
            let obb = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Directions::up()),
            );
            let plane = Plane::new(Directions::up(), P3::new(ZERO, ZERO, ZERO));
            assert_eq!(obb_plane(&obb, &plane), true);
        }
        {
            let obb = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::new(ZERO, -ONE, ZERO)),
            );
            let plane = Plane::new(Directions::up(), P3::new(ZERO, ZERO, ZERO));
            assert_eq!(obb_plane(&obb, &plane), true);
        }
    }
}
