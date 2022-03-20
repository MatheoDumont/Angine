use super::ContactManifold;

use crate::engine::shapes::Sphere;
use crate::math::{vector::*, Real, Vec3, P3};

pub fn sphere_sphere(s1: &Sphere, s2: &Sphere) -> ContactManifold {
    let a2b = &s2.position - &s1.position;
    let l = magnitude(&a2b);
    let penetration_distance = (l - s1.radius - s2.radius).abs() / (2 as Real);
    let n = a2b / l;
    let p = n * (s1.radius - penetration_distance);

    ContactManifold {
        points: vec![p],
        normal_a_to_b: n,
        penetration_distance: penetration_distance,
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::math::{ONE, ZERO};
    #[test]
    fn sphere_sphere_point_intersection() {
        let s1 = Sphere::new(2 as Real, P3::origin());
        let s2 = Sphere::new(1.5 as Real, P3::new(3 as Real, ZERO, ZERO));

        let cm = sphere_sphere(&s1, &s2);

        assert_eq!(cm.penetration_distance, 0.25);
        assert_eq!(cm.points[0][0], 1.75);
        assert_eq!(cm.normal_a_to_b.x(), ONE);
    }
}
