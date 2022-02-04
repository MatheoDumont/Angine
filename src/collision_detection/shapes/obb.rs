use super::{Shape, ShapeType};
use crate::math::{helper, Real, Transform, Vec3, P3};

/**
 * Oriented Bounding Box
 */
pub struct OBB {
    pub half_side: Vec3,
    pub transform: Transform,
}

impl OBB {
    pub fn new(half_side: Vec3, t: Transform) -> OBB {
        OBB {
            half_side: half_side,
            transform: t,
        }
    }

    pub fn min(&self) -> P3 {
        let mut p = P3::from(&self.transform.translation);
        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            p -= axis * self.half_side[i];
        }
        p
    }

    pub fn max(&self) -> P3 {
        let mut p = P3::from(&self.transform.translation);
        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            p += axis * self.half_side[i];
        }
        p
    }

    /**
     * is p inside the OBB
     */
    pub fn is_inside(&self, p: &P3) -> bool {
        let p_to_obb = p - &P3::from(self.transform.translation);
        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            let scalar = &axis.dot(&p_to_obb);

            if scalar > &self.half_side[0] || scalar < &-self.half_side[0] {
                return false;
            }
        }

        true
    }

    /**
     * Closest point on the contour of the OBB to p
     */
    pub fn closest_point(&self, p: &P3) -> P3 {
        let mut new_p = P3::from(&self.transform.translation);
        let p_to_obb = p - &new_p;

        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            let scalar = axis.dot(&p_to_obb);
            new_p += axis * helper::clamp(scalar, -self.half_side[i], self.half_side[i]);
        }

        new_p
    }
}

#[cfg(test)]
mod tests {
    use super::OBB;
    use crate::math::{Real, Rotation, Transform, Vec3, ONE, P3, ZERO};
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn OBB_is_inside() {
        // not rotated
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            // not inside
            {
                let p = P3::new(2 as Real, 2 as Real, 2 as Real);
                assert_eq!(obb.is_inside(&p), false);
            }
            // on the contour
            {
                let p = P3::new(ONE, ONE, ONE);
                assert_eq!(obb.is_inside(&p), true);
            }
            // inside
            {
                let p = P3::new(0.5, 0.5, 0.5);
                assert_eq!(obb.is_inside(&p), true);
            }

            let obb = OBB::new(Vec3::new(ONE, ONE, 2.0), Transform::identity());
            // inside

            let p = P3::new(0.5, 0.5, 3.0);
            assert_eq!(obb.is_inside(&p), false);
        }
        // rotated
        {
            let obb = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4)),
            );
            // not inside
            {
                let p = P3::new(2 as Real, 2 as Real, 2 as Real);
                assert_eq!(obb.is_inside(&p), false);
            }
            // on the contour
            {
                assert_eq!(obb.is_inside(&obb.max()), true);

                assert_eq!(
                    obb.is_inside(&(obb.max() + Vec3::new(0.1, 0.1, 0.1))),
                    false
                );
            }
            // inside
            {
                let p = P3::new(0.2, 0.2, 0.2);
                assert_eq!(obb.is_inside(&p), true);
            }
        }
    }
    #[test]
    fn OBB_closest_point() {
        // not rotated
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let p = obb.closest_point(&P3::new(1.5, 1.5, 0.0));
            assert_eq!(p[0], ONE);
            assert_eq!(p[1], ONE);
            assert_eq!(p[2], ZERO);
        }
        // rotated
        {
            {
                let obb = OBB::new(
                    Vec3::new(ONE, ONE, ONE),
                    Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4)),
                );

                let p = obb.closest_point(&P3::new(0.0, 2.0, 0.0));

                assert_eq!(p[0], ZERO);
                assert_eq!(p[1], (2 as f32).sqrt());
                assert_eq!(p[2], ZERO);
            }

            {
                let obb = OBB::new(
                    Vec3::new(ONE, ONE, 2.0),
                    Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4)),
                );
                let p = obb.closest_point(&P3::new(0.0, 2.0, 2.0));

                assert_eq!(p[0], ZERO);
                assert_eq!(p[1], (2 as f32).sqrt());
                assert_eq!(p[2], 2.0);
            }
        }
    }
}
