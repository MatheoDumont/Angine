use super::ContactInformation;

use crate::engine::shapes::Sphere;
use crate::math::{vector::*, Real, Vec3, P3};

pub fn sphere_sphere(s1: &Sphere, s2: &Sphere) -> ContactInformation {
    let r = s1.radius + s2.radius;
    let mut n = s2.position - s1.position;
    let d = magnitude(&n);
    n /= d;
    
    let penetration_distance = (d - r).abs() * 0.5;
    let dtp = s1.radius - penetration_distance;
    let p = s1.position + n * dtp;
    ContactInformation {
        points: vec![p],
        normal_a_to_b: n,
        penetration_distance,
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::engine::shapes::Shape;
    use crate::math::{ONE, ZERO};
    #[test]
    fn sphere_sphere_point_intersection() {
        let s1 = Sphere::new(2 as Real);
        let mut s2 = Sphere::new(1.5 as Real);
        s2.set_position(P3::new(3 as Real, ZERO, ZERO));

        let cm = sphere_sphere(&s1, &s2);

        assert_eq!(cm.penetration_distance, 0.25);
        assert_eq!(cm.points[0][0], 1.75);
        assert_eq!(cm.normal_a_to_b.x(), ONE);
    }
}
