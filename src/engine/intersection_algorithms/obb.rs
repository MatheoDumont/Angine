use crate::engine::shapes::OBB;
use crate::math::{Vec3, P3};

pub fn obb_obb(obb1: &OBB, obb2: &OBB) -> bool {
    let p = obb1.closest_point(&P3::from(obb2.transform.translation));
    obb2.is_inside(&p)
}

#[cfg(test)]
mod tests {
    use super::obb_obb;
    use crate::engine::shapes::OBB;
    use crate::math::{Real, Rotation, Transform, Vec3, ONE, P3, ZERO};
    #[test]
    fn obb_obb_intersection() {
        // no intersection
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::right() * (3 as Real)),
            );
            assert_eq!(obb_obb(&obb1, &obb2), false);
        }
        // intersection on contour
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::right() * (2 as Real)),
            );
            assert_eq!(obb_obb(&obb1, &obb2), true);
        }
        // big intersect
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::right()),
            );
            assert_eq!(obb_obb(&obb1, &obb2), true);
        }

        // oriented
        {
            let obb1 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4)),
            );
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::right()),
            );
            assert_eq!(obb_obb(&obb1, &obb2), true);
        }
    }
}
