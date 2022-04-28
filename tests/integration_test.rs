extern crate angine;
use angine::engine::{
    collision::collision_object::CollisionObject, shapes, shapes::ShapeType, simulation::RigidBody,
    simulation::SimulationWorld,
};
use angine::math::math_essentials::*;
// use angine::math::Quaternion as AngineQuat;

#[test]
fn init() {
    // Simulation related
    let mut simulation_world = SimulationWorld::new(1.0 / 60.0);
    // add a cube
    let half_length = Vec3::new(1.0, 1.0, 1.0);
    let obb_co = CollisionObject::new(Box::new(shapes::OBB::new(half_length.clone())));
    let obb_rb = RigidBody::new(Transform::identity(), 1.0, false);
    let obb_id = simulation_world.add_rigidbody(obb_rb, obb_co);

    let obb_rb = simulation_world.rigidbody_ref(obb_id);
    assert_eq!(obb_rb.id, 0);
    assert_eq!(obb_rb.collision_object_id(), 0);
    let r = simulation_world
        .collision_world
        .collision_object_ref(obb_rb.collision_object_id());
    assert!(r.is_ok());
    let obb_co = r.unwrap();
    assert_eq!(obb_co.id, 0);
    assert_eq!(obb_co.id_rigid_body, Some(0));
    assert_eq!(obb_co.enabled, true);
    assert_eq!(obb_co.is_static, false);
    assert_eq!(obb_co.shape.shape_type() as usize, ShapeType::OBB as usize);
    assert_eq!(
        obb_rb.transform.translation,
        obb_co.shape.get_transform().translation
    );
    assert_eq!(
        obb_rb.transform.rotation,
        obb_co.shape.get_transform().rotation
    );

    // add a plane
    let plane_co = CollisionObject::new(Box::new(shapes::Plane::new(Directions::up())));
    let plane_rb = RigidBody::new(Transform::translation(P3::new(0.0, -10.0, 0.0)), 1.0, true);
    let plane_id = simulation_world.add_rigidbody(plane_rb, plane_co);

    let plane_rb = simulation_world.rigidbody_ref(plane_id);
    assert_eq!(plane_rb.id, 1);
    assert_eq!(plane_rb.collision_object_id(), 1);
    let r = simulation_world
        .collision_world
        .collision_object_ref(plane_rb.collision_object_id());
    assert!(r.is_ok());
    let plane_co = r.unwrap();
    assert_eq!(plane_co.id, 1);
    assert_eq!(plane_co.id_rigid_body, Some(1));
    assert_eq!(plane_co.enabled, true);
    assert_eq!(plane_co.is_static, true);
    assert_eq!(
        plane_co.shape.shape_type() as usize,
        ShapeType::Plane as usize
    );
    assert_eq!(
        plane_rb.transform.translation,
        plane_co.shape.get_transform().translation
    );
    assert_eq!(
        plane_rb.transform.rotation,
        plane_co.shape.get_transform().rotation
    );
}

#[test]
fn collision_occur() {
    // Simulation related
    let mut simulation_world = SimulationWorld::new(1.0 / 60.0);
    let t = Transform::translation(P3::new(0.0, -10.0, 0.0));
    // add a cube
    let half_length = Vec3::new(1.0, 1.0, 1.0);
    let obb_co = CollisionObject::new(Box::new(shapes::OBB::new(half_length.clone())));
    let obb_rb = RigidBody::new(t.clone(), 1.0, false);
    let obb_id = simulation_world.add_rigidbody(obb_rb, obb_co);

    // add a plane
    let plane_co = CollisionObject::new(Box::new(shapes::Plane::new(Directions::up())));
    let plane_rb = RigidBody::new(t.clone(), 1.0, true);
    let plane_id = simulation_world.add_rigidbody(plane_rb, plane_co);

    simulation_world.collision_world.step();
    assert_eq!(simulation_world.collision_world.contact_manifolds.len(), 1);
}

#[test]

fn not_spining_without_collision() {
    // Simulation related
    let mut simulation_world = SimulationWorld::new(1.0 / 60.0);
    // add a cube
    let half_length = Vec3::new(1.0, 1.0, 1.0);
    let obb_co = CollisionObject::new(Box::new(shapes::OBB::new(half_length.clone())));
    let obb_rb = RigidBody::new(Transform::identity(), 1.0, false);
    let obb_id = simulation_world.add_rigidbody(obb_rb, obb_co);

    let obb_rb = simulation_world.rigidbody_ref(obb_id);
    let position_t = obb_rb.transform.translation;
    let orientation_t = obb_rb.transform.rotation;

    drop(obb_rb);

    simulation_world.discrete_step();
    simulation_world.discrete_step();
    simulation_world.discrete_step();

    let obb_rb = simulation_world.rigidbody_ref(obb_id);

    assert_ne!(position_t, obb_rb.transform.translation);
    assert_eq!(orientation_t, obb_rb.transform.rotation);
}
