use crate::engine::shapes::Plane;
use crate::math::math_essentials::*;

pub fn plane_plane(pl1: &Plane, pl2: &Plane) -> bool {
    if dot(&pl1.normal, &pl2.normal) == ONE && pl1.distance_from_origin != pl2.distance_from_origin
    {
        false
    } else {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::plane_plane;
    use crate::engine::shapes::Plane;
    use crate::engine::shapes::Shape;
    use crate::math::math_essentials::*;
    #[test]
    fn plane_plane_intersection() {
        {
            let pl1 = Plane::new(Directions::up());
            let mut pl2 = Plane::new(Directions::up());

            assert_eq!(plane_plane(&pl1, &pl2), true);

            pl2.distance_from_origin = ONE;
            assert_eq!(plane_plane(&pl1, &pl2), false);
        }

        {
            let pl1 = Plane::new(Directions::up());
            let mut pl2 = Plane::new(normalized(Vec3::new(ONE, ONE, ZERO)));
            pl2.set_position(P3::new(ZERO, ONE, ZERO));

            assert_eq!(plane_plane(&pl1, &pl2), true);
        }
    }
}
