use super::shapes::Sphere;
// use crate::collision_detection::intersection_wrapper::Intersection;
use crate::math::{Point, Real};

pub fn sphere_sphere<'a>(a: &'a Point, sphereA: &Sphere, b: &'a Point, sphereB: &Sphere) -> bool {
    let distance_a2b = (a - b).norm();
    let r = sphereA.radius + sphereB.radius >= distance_a2b
        && (sphereA.radius - sphereB.radius).abs() < distance_a2b;
    r
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sphere_sphere_intersect() {
        let p1 = Point::origin();
        let p2 = Point::new(1 as Real, 0 as Real, 0 as Real);

        let x = sphere_sphere(&p1, &Sphere::new(1.0), &p2, &Sphere::new(1.5));

        assert!(x);
    }

    #[test]
    fn sphere_sphere_doesnt_intersect() {
        let p1 = Point::origin();
        let p2 = Point::new(3 as Real, 0 as Real, 0 as Real);

        let x = sphere_sphere(&p1, &Sphere::new(1.0), &p2, &Sphere::new(1.0));

        assert!(x);
    }

    #[test]
    fn sphere_sphere_intersect_one_point() {
        let p1 = Point::origin();
        let p2 = Point::new(2 as Real, 0 as Real, 0 as Real);

        let x = sphere_sphere(&p1, &Sphere::new(1.0), &p2, &Sphere::new(1.0));

        assert!(x);
    }

    #[test]
    fn sphere_sphere_one_inside_another_dont_intersect() {
        let p1 = Point::origin();
        let p2 = Point::new(1 as Real, 0 as Real, 0 as Real);

        let x = sphere_sphere(&p1, &Sphere::new(1.0), &p2, &Sphere::new(2.0));

        assert!(x);
    }
}
