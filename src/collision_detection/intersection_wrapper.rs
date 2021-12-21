use crate::collision_detection::collisions::{CollisionObject, Shape, ShapeType, Sphere};
use crate::collision_detection::intersection_algorithm;
use crate::math::{point::Point, vector::Vector};

pub struct Intersection {
    pub point: Point,
    pub normal_a_to_b: Vector,
}

// pub trait IntersectionAlgoHandler<T, V>
// where
//     T: Shape,
//     V: Shape,
// {
//     fn compute(
//         &self,
//         shape1: (&'static Box<dyn Shape>, &'static Point),
//         shape2: (&'static Box<dyn Shape>, &'static Point),
//     ) -> Option<Intersection> {
//         let s1 = shape1.0.downcast_ref::<T>().expect("Tried to downcast");
//         let p1 = shape1.1;
//         let s2 = shape2.0.downcast_ref::<V>().expect("Tried to downcast");
//         let p2 = shape2.1;

//         self.private_compute(p1, s1, p2, s2)
//     }

//     fn private_compute(
//         &self,
//         p1: &'static Point,
//         s1: &'static T,
//         p2: &'static Point,
//         s2: &'static V,
//     ) -> Option<Intersection>;
// }

// struct SphereToSphere;
// impl IntersectionAlgoHandler<Sphere, Sphere> for SphereToSphere {
//     fn private_compute(
//         &self,
//         p1: &'static Point,
//         s1: &'static Sphere,
//         p2: &'static Point,
//         s2: &'static Sphere,
//     ) -> Option<Intersection> {
//         intersection_algorithm::sphere_sphere(p1, s1.radius, p2, s2.radius)
//     }
// }

pub trait IntersectionAlgoHandler {
    fn compute(
        &self,
        p1: &'static Point,
        s1: &Box<dyn Shape>,
        p2: &'static Point,
        s2: &Box<dyn Shape>,
    ) -> Option<Intersection>;
}

struct SphereToSphere;

impl IntersectionAlgoHandler for SphereToSphere {
    fn compute(
        &self,
        p1: &'static Point,
        shape1: &Box<dyn Shape>,
        p2: &'static Point,
        shape2: &Box<dyn Shape>,
    ) -> Option<Intersection> {
        let sphere1 = shape1
            .downcast_ref::<Sphere>()
            .expect("Tried to downcast to Sphere");
        let sphere2 = shape2
            .downcast_ref::<Sphere>()
            .expect("Tried to downcast to Sphere");

        intersection_algorithm::sphere_sphere(p1, sphere1.radius, p2, sphere2.radius)
    }
}

pub fn get_intersection_fn_by_collisiontypes(
    c1: &'static CollisionObject,
    c2: &'static CollisionObject,
) -> Option<impl IntersectionAlgoHandler> {
    match (c1.shape.collision_type(), c2.shape.collision_type()) {
        (ShapeType::Sphere, ShapeType::Sphere) => Some(SphereToSphere),
        (_, _) => None,
    }
    
}
