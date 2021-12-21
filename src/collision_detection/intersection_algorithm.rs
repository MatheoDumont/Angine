use crate::collision_detection::intersection_wrapper::Intersection;
use crate::math::{point::Point, Real};

pub fn sphere_sphere<'a>(p1: &'a Point, r1: Real, p2: &'a Point, r2: Real) -> Option<Intersection> {
    // on considère que l'intersection se fait en un seul point plutot que deux
    // pour plus de simplicité, meilleure efficience, stabilité (à vérifier)
    let d = p1 - p2;
    let l = d.norm();
    if r1 + r2 >= l && (r1 - r2).abs() < l {
        let normal_a_to_b = d / l;
        let p = p1 + &normal_a_to_b * r1;

        Some(Intersection {
            point: p,
            normal_a_to_b: normal_a_to_b,
        })
    } else {
        None
    }
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

        assert!(x.is_some());
    }

    #[test]
    fn sphere_sphere_doesnt_intersect() {
        let p1 = Point::origin();
        let r1 = 1 as Real;

        let p2 = Point::new(3 as Real, 0 as Real, 0 as Real);
        let r2 = 1 as Real;

        let x = sphere_sphere(&p1, r1, &p2, r2);

        assert!(x.is_none());
    }

    #[test]
    fn sphere_sphere_intersect_one_point() {
        let p1 = Point::origin();
        let r1 = 1 as Real;

        let p2 = Point::new(2 as Real, 0 as Real, 0 as Real);
        let r2 = 1 as Real;

        let x = sphere_sphere(&p1, r1, &p2, r2);

        assert!(x.is_some());
    }

    #[test]
    fn sphere_sphere_one_inside_another_dont_intersect() {
        let p1 = Point::origin();
        let r1 = 1 as Real;

        let p2 = Point::new(1 as Real, 0 as Real, 0 as Real);
        let r2 = 2 as Real;

        let x = sphere_sphere(&p1, r1, &p2, r2);

        assert!(x.is_none());
    }
}
