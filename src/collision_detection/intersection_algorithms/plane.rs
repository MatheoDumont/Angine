use crate::collision_detection::shapes::Plane;
use crate::math::ONE;

pub fn plane_plane(pl1: &Plane, pl2: &Plane) -> bool {
    if pl1.normal.dot(&pl2.normal) == ONE && pl1.distance_from_origin != pl2.distance_from_origin {
        false
    } else {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::plane_plane;
    use crate::collision_detection::shapes::Plane;
    use crate::math::{Vec3, ONE, P3, ZERO};
    #[test]
    fn plane_plane_intersection() {
        {
            let pl1 = Plane::new(Vec3::up(), P3::origin());
            let mut pl2 = Plane::new(Vec3::up(), P3::origin());

            assert_eq!(plane_plane(&pl1, &pl2), true);

            pl2.distance_from_origin = ONE;
            assert_eq!(plane_plane(&pl1, &pl2), false);
        }

        {
            let pl1 = Plane::new(Vec3::up(), P3::origin());
            let pl2 = Plane::new(
                Vec3::new(ONE, ONE, ZERO).normalized(),
                P3::new(ZERO, ONE, ZERO),
            );
            assert_eq!(plane_plane(&pl1, &pl2), true);
        }
    }
}
