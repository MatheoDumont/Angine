use crate::collision_detection::collision::CollisionObject;

use crate::collision_detection::shapes::{Shape, ShapeType, Sphere};
use crate::math::{Vec3, P3};

pub trait IntersectionAlgoHandler {
    fn compute(&self, s1: &Box<dyn Shape>, s2: &Box<dyn Shape>) -> bool;
}

struct SphereToSphere;

impl IntersectionAlgoHandler for SphereToSphere {
    fn compute(&self, shape1: &Box<dyn Shape>, shape2: &Box<dyn Shape>) -> bool {
        let sphere1 = shape1
            .downcast_ref::<Sphere>()
            .expect("Tried to downcast to Sphere");
        let sphere2 = shape2
            .downcast_ref::<Sphere>()
            .expect("Tried to downcast to Sphere");

        super::sphere::sphere_sphere(sphere1, sphere2)
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
