use crate::engine::shapes::Sphere;
use crate::math::{Real, P3};

pub fn sphere_sphere(sphere_a: &Sphere, sphere_b: &Sphere) -> bool {
    let distance_a2b = (sphere_a.position - sphere_b.position).norm();
    let r = sphere_a.radius + sphere_b.radius >= distance_a2b
        && (sphere_a.radius - sphere_b.radius).abs() < distance_a2b;
    r
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sphere_sphere_intersection() {
        {
            let p1 = P3::origin();
            let p2 = P3::new(1 as Real, 0 as Real, 0 as Real);

            let x = sphere_sphere(&Sphere::new(1.0, p1), &Sphere::new(1.5, p2));

            assert_eq!(x, true);
        }

        //sphere_sphere_one_inside_another_dont_intersect
        {
            let p1 = P3::origin();
            let p2 = P3::new(1 as Real, 0 as Real, 0 as Real);

            let x = sphere_sphere(&Sphere::new(1.0, p1), &Sphere::new(2.0, p2));

            assert_eq!(x, false);
        }

        //sphere_sphere_doesnt_intersect
        {
            let p1 = P3::origin();
            let p2 = P3::new(3 as Real, 0 as Real, 0 as Real);
            let x = sphere_sphere(&Sphere::new(1.0, p1), &Sphere::new(1.0, p2));

            assert_eq!(x, false);
        }

        //sphere_sphere_intersect_one_point
        {
            let p1 = P3::origin();
            let p2 = P3::new(2 as Real, 0 as Real, 0 as Real);

            let x = sphere_sphere(&Sphere::new(1.0, p1), &Sphere::new(1.0, p2));

            assert_eq!(x, true);
        }
    }
}
