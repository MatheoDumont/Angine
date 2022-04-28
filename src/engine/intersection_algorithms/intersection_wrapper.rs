use crate::engine::contact_algorithms;
use crate::engine::contact_algorithms::ContactInformation;
use crate::engine::shapes::{Plane, Shape, Sphere, OBB};
use crate::math::Vec3;

type FuncType = fn(&Box<dyn Shape>, &Box<dyn Shape>) -> Option<ContactInformation>;
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

/**
 * Dans les cas ou les arguments shape sont dans un sens et sont intervertis car la fonction d'intersection
 * et de contact les prends dans l'autre sens,dans ce cas on swap la normal pour qu'elle pointe toujours de a vers b.
 *
 * Exemple:
 * get_intersection_fn_by_collisiontypes(sphere, OBB)
 * i1 = 0, i2 = 1
 * et dans le tableau INTERSECTIONS_FUNCTIONS_BY_SHAPE_TYPE[0][1] = compute_sphere_obb
 * MAIS !
 * 'compute_sphere_obb' est une fonction de facade pour accélérer la le recherche de la fonction d'intersection
 * la vraie fonction qui calcule l'intersection est 'obb_sphere::obb_sphere(o2, o1)' et prend les arguments
 * dans l'autre ordre donc quand cela arrive il fois swap la normal.
 *
 * Pourquoi on ne passe pas le CollisionObject directement : détail d'implementation, je voulais séparer les données vu
 * qu'on a besoin que de la shape et d'une position, à terme ca pourrait changer si la position se retrouve dans CollisionObject.
 * C'est le genre de détail chiant de conception qui se prévoit mal en amont car j'avais pas une idée claire de à quoi je voulais que ca
 * ressemble.
 */
pub fn swap_normal_orientation(normal: &mut Vec3) {
    *normal = -*normal;
}

fn compute_sphere_sphere(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactInformation> {
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

fn compute_obb_obb(shape1: &Box<dyn Shape>, shape2: &Box<dyn Shape>) -> Option<ContactInformation> {
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
) -> Option<ContactInformation> {
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

fn compute_sphere_obb(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactInformation> {
    let o1 = shape1
        .downcast_ref::<Sphere>()
        .expect("Tried to downcast to Sphere");
    let o2 = shape2
        .downcast_ref::<OBB>()
        .expect("Tried to downcast to OBB");

    if super::obb_sphere::obb_sphere(o2, o1) {
        let mut ci = contact_algorithms::obb_sphere(o2, o1);
        swap_normal_orientation(&mut ci.normal_a_to_b);
        Some(ci)
    } else {
        None
    }
}

fn compute_obb_sphere(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactInformation> {
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

fn compute_obb_plane(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactInformation> {
    let o1 = shape1
        .downcast_ref::<OBB>()
        .expect("Tried to downcast to OBB");
    let o2 = shape2
        .downcast_ref::<Plane>()
        .expect("Tried to downcast to Plane");

    if super::obb_plane::obb_plane(o1, o2) {
        println!("collsion obb plane");
        Some(contact_algorithms::obb_plane(o1, o2))
    } else {
        None
    }
}

fn compute_plane_obb(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactInformation> {
    let o1 = shape1
        .downcast_ref::<Plane>()
        .expect("Tried to downcast to Plane");
    let o2 = shape2
        .downcast_ref::<OBB>()
        .expect("Tried to downcast to OBB");

    if super::obb_plane::obb_plane(o2, o1) {
        println!("collsion plane obb");

        let mut ci = contact_algorithms::obb_plane(o2, o1);
        swap_normal_orientation(&mut ci.normal_a_to_b);
        Some(ci)
    } else {
        None
    }
}

fn compute_plane_sphere(
    shape1: &Box<dyn Shape>,
    shape2: &Box<dyn Shape>,
) -> Option<ContactInformation> {
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
) -> Option<ContactInformation> {
    let o1 = shape1
        .downcast_ref::<Sphere>()
        .expect("Tried to downcast to Sphere");
    let o2 = shape2
        .downcast_ref::<Plane>()
        .expect("Tried to downcast to Plane");

    if super::plane_sphere::plane_sphere(o2, o1) {
        let mut ci = contact_algorithms::plane_sphere(o2, o1);
        swap_normal_orientation(&mut ci.normal_a_to_b);
        Some(ci)
    } else {
        None
    }
}
