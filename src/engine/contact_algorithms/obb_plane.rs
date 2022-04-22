use super::ContactInformation;
use crate::engine::shapes::{Plane, OBB};
use crate::math::math_essentials::*;

pub fn obb_plane(obb: &OBB, plane: &Plane) -> ContactInformation {
    let point_to_plane = plane.point_to_plane(&obb.transform.position());
    let direction = normalized(&point_to_plane);
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
            let obb = OBB::new(
                Vec3::value(ONE),
                Transform::translation(Vec3::new(ZERO, 0.5, ZERO)),
            );
            let plane = Plane::new(Directions::up(), Vec3::origin());

            let res = obb_plane(&obb, &plane);
            assert_eq!(res.points[0].x(), ZERO);
            assert_eq!(res.points[0].y(), -0.5);
            assert_eq!(res.points[0].z(), ZERO);
        }
        {
            let r = Rotation::composed(ZERO, ZERO, helper::angle_2_rad(45.0));
            let t = Transform::new(Vec3::value(ONE), r, Directions::up());
            let p = P3::new(-ONE, -ONE, -ONE);

            let p = t.transform(&p);

            let obb = OBB::new(Vec3::value(ONE), t);

            let plane = Plane::new(Directions::up(), Vec3::origin());

            let res = obb_plane(&obb, &plane);

            assert_approx_eq!(res.points[0].x(), p.x());
            assert_approx_eq!(res.points[0].y(), p.y());
            assert_approx_eq!(res.points[0].z(), ZERO);
        }
    }
}
