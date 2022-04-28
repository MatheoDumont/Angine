use crate::engine::shapes::Sphere;
use crate::math::math_essentials::*;

pub fn sphere_sphere(sphere_a: &Sphere, sphere_b: &Sphere) -> bool {
    let d = sphere_a.position - sphere_b.position;
    let distance_a2b = magnitude(&d);
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
            let s1 = Sphere::new(1.0);
            let mut s2 = Sphere::new(1.5);
            s2.position = P3::new(1 as Real, 0 as Real, 0 as Real);

            assert_eq!(sphere_sphere(&s1, &s2), true);
        }

        //sphere_sphere_one_inside_another_dont_intersect
        {
            let s1 = Sphere::new(1.0);
            let mut s2 = Sphere::new(2.0);
            s2.position = P3::new(1 as Real, 0 as Real, 0 as Real);

            assert_eq!(sphere_sphere(&s1, &s2), false);
        }

        //sphere_sphere_doesnt_intersect
        {
            let s1 = Sphere::new(1.0);
            let mut s2 = Sphere::new(1.0);
            s2.position = P3::new(3 as Real, 0 as Real, 0 as Real);

            assert_eq!(sphere_sphere(&s1, &s2), false);
        }

        //sphere_sphere_intersect_one_point
        {
            let s1 = Sphere::new(1.0);
            let mut s2 = Sphere::new(1.0);
            s2.position = P3::new(2 as Real, 0 as Real, 0 as Real);

            assert_eq!(sphere_sphere(&s1, &s2), true);
        }
    }
}
