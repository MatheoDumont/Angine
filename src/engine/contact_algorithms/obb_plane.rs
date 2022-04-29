use super::ContactInformation;
use crate::engine::shapes::{Plane, Shape, OBB};
use crate::math::math_essentials::*;

pub fn obb_plane(obb: &OBB, plane: &Plane) -> ContactInformation {
    let point_on_plane = plane.reject_point_on_plane(&obb.transform.position());

    let direction = normalized(&(&point_on_plane - obb.get_position()));
    let p = obb.project_on_contour_in_direction(&direction);

    ContactInformation {
        points: vec![p],
        normal_a_to_b: direction,
        penetration_distance: plane.signed_distance(&p).abs(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    #[test]
    fn test_obb_plane_contact() {
        {
            let mut obb = OBB::new(Vec3::value(ONE));
            obb.transform = Transform::translation(Vec3::new(ZERO, 0.5, ZERO));
            let plane = Plane::new(Directions::up());

            let res = obb_plane(&obb, &plane);
            assert_eq!(res.points[0].x(), ZERO);
            assert_eq!(res.points[0].y(), -0.5);
            assert_eq!(res.points[0].z(), ZERO);

            assert_eq!(res.normal_a_to_b.x(), ZERO);
            assert_eq!(res.normal_a_to_b.y(), -ONE);
            assert_eq!(res.normal_a_to_b.z(), ZERO);

            assert_eq!(res.penetration_distance, 0.5);
        }
        {
            let r = Rotation::Z(helper::angle_2_rad(45.0));
            let t = Transform::new(Vec3::value(ONE), r, Directions::up());
            let p = P3::new(-ONE, -ONE, -ONE);

            let p = t.transform(&p);

            let mut obb = OBB::new(Vec3::value(ONE));
            obb.transform = t;

            let plane = Plane::new(Directions::up());

            let res = obb_plane(&obb, &plane);

            assert_approx_eq!(res.points[0].x(), p.x());
            assert_approx_eq!(res.points[0].y(), p.y());
            assert_approx_eq!(res.points[0].z(), ZERO);

            assert!(
                dot(
                    &res.normal_a_to_b,
                    &(plane.get_position() - &obb.transform.translation)
                ) > ZERO
            );
            assert_approx_eq!(
                res.penetration_distance,
                magnitude(&(r * P3::new(ONE, ONE, ZERO))) - ONE
            );
        }

        {
            let mut obb = OBB::new(Vec3::value(ONE));
            obb.transform.translation = Vec3::new(ZERO, -9.0, ZERO);
            let mut plane = Plane::new(Directions::up());
            plane.set_transform(Transform::translation(Vec3::new(ZERO, -10.0, ZERO)));

            let res = obb_plane(&obb, &plane);

            assert_eq!(res.points[0].x(), ZERO);
            assert_eq!(res.points[0].y(), -10.0);
            assert_eq!(res.points[0].z(), ZERO);
            assert_eq!(res.penetration_distance, ZERO);
        }
    }
}
