use crate::collision_detection::intersection_wrapper::Intersection;
use crate::math::{point::Point, Real};

pub fn sphere_sphere<'a>(a: &'a Point, radiusA: Real, b: &'a Point, radiusB: Real) -> bool {
    let distance_a2b = (a - b).norm();
    let r = radiusA + radiusB >= distance_a2b && (radiusA - radiusB).abs() < distance_a2b;
    r
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sphere_sphere_intersect() {
        let p1 = Point::origin();
        let r1 = 1 as Real;

        let p2 = Point::new(1 as Real, 0 as Real, 0 as Real);
        let r2 = 1.5 as Real;

        let x = sphere_sphere(&p1, r1, &p2, r2);

        assert!(x);
    }

    #[test]
    fn sphere_sphere_doesnt_intersect() {
        let p1 = Point::origin();
        let r1 = 1 as Real;

        let p2 = Point::new(3 as Real, 0 as Real, 0 as Real);
        let r2 = 1 as Real;

        let x = sphere_sphere(&p1, r1, &p2, r2);

        assert!(x);
    }

    #[test]
    fn sphere_sphere_intersect_one_point() {
        let p1 = Point::origin();
        let r1 = 1 as Real;

        let p2 = Point::new(2 as Real, 0 as Real, 0 as Real);
        let r2 = 1 as Real;

        let x = sphere_sphere(&p1, r1, &p2, r2);

        assert!(x);
    }

    #[test]
    fn sphere_sphere_one_inside_another_dont_intersect() {
        let p1 = Point::origin();
        let r1 = 1 as Real;

        let p2 = Point::new(1 as Real, 0 as Real, 0 as Real);
        let r2 = 2 as Real;

        let x = sphere_sphere(&p1, r1, &p2, r2);

        assert!(x);
    }
}
