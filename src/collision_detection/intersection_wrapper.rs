use crate::collision_detection::collision::CollisionObject;
use crate::collision_detection::intersection_algorithm;
use crate::collision_detection::shapes::{Shape, ShapeType, Sphere};
use crate::math::{point::Point, vector::Vector};



pub trait IntersectionAlgoHandler {
    fn compute(
        &self,
        p1: &'static Point,
        s1: &Box<dyn Shape>,
        p2: &'static Point,
        s2: &Box<dyn Shape>,
    ) -> bool;
}

struct SphereToSphere;

impl IntersectionAlgoHandler for SphereToSphere {
    fn compute(
        &self,
        p1: &'static Point,
        shape1: &Box<dyn Shape>,
        p2: &'static Point,
        shape2: &Box<dyn Shape>,
    ) -> bool {
        let sphere1 = shape1
            .downcast_ref::<Sphere>()
            .expect("Tried to downcast to Sphere");
        let sphere2 = shape2
            .downcast_ref::<Sphere>()
            .expect("Tried to downcast to Sphere");

        intersection_algorithm::sphere_sphere(p1, sphere1, p2, sphere2)
    }
}

pub fn get_intersection_fn_by_collisiontypes(
    c1: &'static CollisionObject,
    c2: &'static CollisionObject,
) -> Option<impl IntersectionAlgoHandler> {
    match (c1.shape.shape_type(), c2.shape.shape_type()) {
        (ShapeType::Sphere, ShapeType::Sphere) => Some(SphereToSphere),
        (_, _) => None,
    }
}
