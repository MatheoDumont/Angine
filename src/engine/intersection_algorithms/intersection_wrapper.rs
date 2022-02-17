use crate::engine::collision::CollisionObject;
use crate::engine::contact_algorithms;
use crate::engine::contact_algorithms::ContactManifold;
use crate::engine::shapes::{Plane, Shape, Sphere, OBB};

type FuncType = fn(&Box<dyn Shape>, &Box<dyn Shape>) -> Option<ContactManifold>;
const N_SHAPES: usize = 3;
const INTERSECTIONS_FUNCTIONS_BY_SHAPE_TYPE: [[Option<FuncType>; N_SHAPES]; N_SHAPES] = [
    //Sphere = 0
    [
        Some(compute_sphere_sphere),
        Some(compute_sphere_obb),
        Some(compute_sphere_plane),
    ],
    // OBB = 1
    [
        Some(compute_obb_sphere),
        Some(compute_obb_obb),
        Some(compute_obb_plane),
    ],
    // Plane = 2
    [
        Some(compute_plane_sphere),
        Some(compute_plane_obb),
        Some(compute_plane_plane),
    ],
];

/**
 * Return the intersection algorithm corresponding to the shapes of the CollisionObjects.
 * The algorithms depends on the order of the arguments order given to `get_intersection_fn_by_collisiontypes` and must not be changed.
 *
 * Example:
 * let c1 = CollisionObject(Shape::new(ShapeType::Sphere));
 * let c2 = CollisionObject(Shape::new(ShapeType::Plane));
 *
 * let algo = get_intersection_fn_by_collisiontypes(c1,c2);
 * ---> algo(c1, c2) and not algo(c2, c1)
 *
 */
pub fn get_intersection_fn_by_collisiontypes(
    s1: &Box<dyn Shape>,
    s2: &Box<dyn Shape>,
) -> Option<FuncType> {
    let i1 = s1.as_ref().shape_type() as usize;
    let i2 = s2.as_ref().shape_type() as usize;
    INTERSECTIONS_FUNCTIONS_BY_SHAPE_TYPE[i1][i2]
}

fn compute_sphere_sphere(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactManifold> {
    let o1 = shape1
        .downcast_ref::<Sphere>()
        .expect("Tried to downcast to Sphere");
    let o2 = shape2
        .downcast_ref::<Sphere>()
        .expect("Tried to downcast to Sphere");

    if super::sphere::sphere_sphere(o1, o2) {
        Some(contact_algorithms::sphere_sphere(o1, o2))
    } else {
        None
    }
}

fn compute_obb_obb(shape1: &Box<dyn Shape>, shape2: &Box<dyn Shape>) -> Option<ContactManifold> {
    let o1 = shape1
        .downcast_ref::<OBB>()
        .expect("Tried to downcast to OBB");
    let o2 = shape2
        .downcast_ref::<OBB>()
        .expect("Tried to downcast to OBB");

    if super::obb::obb_obb(o1, o2) {
        Some(contact_algorithms::obb_obb(o1, o2))
    } else {
        None
    }
}

fn compute_plane_plane(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactManifold> {
    let o1 = shape1
        .downcast_ref::<Plane>()
        .expect("Tried to downcast to Plane");
    let o2 = shape2
        .downcast_ref::<Plane>()
        .expect("Tried to downcast to Plane");

    if super::plane::plane_plane(o1, o2) {
        Some(contact_algorithms::plane_plane(o1, o2))
    } else {
        None
    }
}

fn compute_sphere_obb(shape1: &Box<dyn Shape>, shape2: &Box<dyn Shape>) -> Option<ContactManifold> {
    let o1 = shape1
        .downcast_ref::<Sphere>()
        .expect("Tried to downcast to Sphere");
    let o2 = shape2
        .downcast_ref::<OBB>()
        .expect("Tried to downcast to OBB");

    if super::obb_sphere::obb_sphere(o2, o1) {
        Some(contact_algorithms::obb_sphere(o2, o1))
    } else {
        None
    }
}

fn compute_obb_sphere(shape1: &Box<dyn Shape>, shape2: &Box<dyn Shape>) -> Option<ContactManifold> {
    let o1 = shape1
        .downcast_ref::<OBB>()
        .expect("Tried to downcast to OBB");
    let o2 = shape2
        .downcast_ref::<Sphere>()
        .expect("Tried to downcast to Sphere");

    if super::obb_sphere::obb_sphere(o1, o2) {
        Some(contact_algorithms::obb_sphere(o1, o2))
    } else {
        None
    }
}

fn compute_obb_plane(shape1: &Box<dyn Shape>, shape2: &Box<dyn Shape>) -> Option<ContactManifold> {
    let o1 = shape1
        .downcast_ref::<OBB>()
        .expect("Tried to downcast to OBB");
    let o2 = shape2
        .downcast_ref::<Plane>()
        .expect("Tried to downcast to Plane");

    if super::obb_plane::obb_plane(o1, o2) {
        Some(contact_algorithms::obb_plane(o1, o2))
    } else {
        None
    }
}

fn compute_plane_obb(shape1: &Box<dyn Shape>, shape2: &Box<dyn Shape>) -> Option<ContactManifold> {
    let o1 = shape1
        .downcast_ref::<Plane>()
        .expect("Tried to downcast to Plane");
    let o2 = shape2
        .downcast_ref::<OBB>()
        .expect("Tried to downcast to OBB");

    if super::obb_plane::obb_plane(o2, o1) {
        Some(contact_algorithms::obb_plane(o2, o1))
    } else {
        None
    }
}

fn compute_plane_sphere(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactManifold> {
    let o1 = shape1
        .downcast_ref::<Plane>()
        .expect("Tried to downcast to Plane");
    let o2 = shape2
        .downcast_ref::<Sphere>()
        .expect("Tried to downcast to Sphere");

    if super::plane_sphere::plane_sphere(o1, o2) {
        Some(contact_algorithms::plane_sphere(o1, o2))
    } else {
        None
    }
}

fn compute_sphere_plane(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactManifold> {
    let o1 = shape1
        .downcast_ref::<Sphere>()
        .expect("Tried to downcast to Sphere");
    let o2 = shape2
        .downcast_ref::<Plane>()
        .expect("Tried to downcast to Plane");

    if super::plane_sphere::plane_sphere(o2, o1) {
        Some(contact_algorithms::plane_sphere(o2, o1))
    } else {
        None
    }
}
