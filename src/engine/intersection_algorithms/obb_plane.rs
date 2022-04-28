use crate::engine::shapes::{Plane, OBB};
use crate::math::math_essentials::*;

pub fn obb_plane(obb: &OBB, plane: &Plane) -> bool {
    let rotation = &obb.transform.rotation;

    // distance of the closest point from the obb to the plane
    let closest_distance = obb.half_side.x() * dot(&plane.normal, &rotation.row(0)).abs()
        + obb.half_side.y() * dot(&plane.normal, &rotation.row(1)).abs()
        + obb.half_side.z() * dot(&plane.normal, &rotation.row(2)).abs();
    println!(
        "{:?} {:?} {:?}",
        dot(&plane.normal, &obb.transform.translation),
        plane.signed_distance(&obb.transform.translation),
        closest_distance
    );
    plane.signed_distance(&obb.transform.translation).abs() <= closest_distance
}

#[cfg(test)]
mod tests {
    use super::obb_plane;
    use crate::engine::shapes::{Plane, Shape, OBB};
    use crate::math::math_essentials::*;

    #[test]
    fn obb_plane_intersection() {
        // no intersection
        {
            let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
            obb.transform = Transform::translation(Vec3::new(ZERO, 2 as Real, ZERO));
            let plane = Plane::new(Directions::up());
            assert_eq!(obb_plane(&obb, &plane), false);
        }
        // rotated obb no intersection
        {
            let mut obb = OBB::new(Vec3::new(0.6, 0.6, 0.6));
            obb.transform = Transform::new(
                Vec3::zeros(),
                Rotation::Z(std::f32::consts::FRAC_PI_4),
                Vec3::new(ZERO, 1.5, ZERO),
            );
            let plane = Plane::new(Rotation::Z(std::f32::consts::FRAC_PI_4) * Directions::up());
            assert_eq!(obb_plane(&obb, &plane), false);
        }

        // rotated obb no intersection moved along z
        {
            let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
            obb.transform = Transform::new(
                Vec3::zeros(),
                Rotation::Z(std::f32::consts::FRAC_PI_4),
                Vec3::new(ZERO, 1.5, 5 as Real),
            );
            let plane = Plane::new(Directions::up());
            assert_eq!(obb_plane(&obb, &plane), false);
        }

        // intersection
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE));
            let mut plane = Plane::new(Directions::up());
            plane.set_position(P3::new(ZERO, -ONE, ZERO));
            assert_eq!(obb_plane(&obb, &plane), true);
        }
        // origin above, pt below plane
        {
            let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
            obb.transform = Transform::new(
                Vec3::ones(),
                Rotation::Z(std::f32::consts::FRAC_PI_4),
                Vec3::new(ZERO, ONE, ZERO),
            );
            let plane = Plane::new(Directions::up());
            assert_eq!(obb_plane(&obb, &plane), true);
        }
        // intersection on contour
        {
            let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
            obb.transform = Transform::translation(Directions::up());
            let plane = Plane::new(Directions::up());
            assert_eq!(obb_plane(&obb, &plane), true);
        }
        {
            let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
            obb.transform = Transform::translation(Vec3::new(ZERO, -ONE, ZERO));
            let plane = Plane::new(Directions::up());
            assert_eq!(obb_plane(&obb, &plane), true);
        }

        {
            let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
            // obb.transform = Transform::translation(Directions::up());
            let mut plane = Plane::new(Directions::up());
            plane.set_transform(Transform::translation(Vec3::new(0.0, -10.0, 0.0)));

            assert_eq!(obb_plane(&obb, &plane), false);

            obb.transform = Transform::translation(Vec3::new(0.0, -10.0, 0.0));
            assert_eq!(obb_plane(&obb, &plane), true);
        }
    }
}
