extern crate angine;
use angine::engine::{
    collision::collision_object::CollisionObject, collision::CollisionWorld, shapes, shapes::Shape,
    shapes::ShapeType, simulation::RigidBody, simulation::SimulationWorld,
};
use angine::math::math_essentials::*;
// use angine::math::Quaternion as AngineQuat;

#[test]
fn init() {
    // Simulation related
    let mut simulation_world = SimulationWorld::new(None);
    // add a cube
    let half_length = Vec3::new(1.0, 1.0, 1.0);
    let obb_co = CollisionObject::new(Box::new(shapes::OBB::new(half_length.clone())));
    let obb_rb = RigidBody::new(Transform::identity(), 1.0, false);
    let obb_id = simulation_world.add_rigidbody(obb_rb, obb_co);

    let obb_rb = simulation_world.rigidbody_ref(obb_id);
    assert_eq!(obb_rb.id, 0);
    assert_eq!(obb_rb.collision_object_id(), 0);
    assert_eq!(obb_rb.mass(), 1.0);
    let r = simulation_world
        .collision_world
        .collision_object_ref(obb_rb.collision_object_id());
    assert!(r.is_ok());
    assert_eq!(obb_rb.center_of_mass(), &obb_rb.transform.position());
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
    let mut simulation_world = SimulationWorld::new(None);
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
fn changes_in_sim() {
    // Simulation related
    let mut simulation_world = SimulationWorld::new(None);
    // add a cube
    let half_length = Vec3::new(1.0, 1.0, 1.0);
    let obb_co = CollisionObject::new(Box::new(shapes::OBB::new(half_length.clone())));
    let obb_rb = RigidBody::new(Transform::identity(), 1.0, false);
    let obb_id = simulation_world.add_rigidbody(obb_rb, obb_co);

    simulation_world.discrete_step();
    simulation_world.discrete_step();
    simulation_world.discrete_step();

    let obb_rb = simulation_world.rigidbody_ref(obb_id);

    assert_eq!(obb_rb.center_of_mass(), &obb_rb.transform.translation);
}

#[test]
fn not_spining_without_collision() {
    // Simulation related
    let mut simulation_world = SimulationWorld::new(None);
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

#[test]
fn assert_on_collision() {
    let mut collision_world = CollisionWorld::new();
    // obb
    let mut obb_co = CollisionObject::new(Box::new(shapes::OBB::new(Vec3::new(1.0, 1.0, 1.0))));
    obb_co
        .shape
        .set_transform(Transform::translation(Directions::up() * 0.5));
    let obb_id = collision_world.add_collision_object(obb_co);
    // plane
    let plane_co = CollisionObject::new(Box::new(shapes::Plane::new(Directions::up())));
    let plane_id = collision_world.add_collision_object(plane_co);

    let r = collision_world.are_colliding(obb_id, plane_id);
    assert!(r.is_some());
    let cm = r.unwrap();
    let a = collision_world
        .collision_object_ref(cm.id_collision_object_a)
        .unwrap();
    let b = collision_world
        .collision_object_ref(cm.id_collision_object_b)
        .unwrap();
    let ab = b.shape.get_position() - a.shape.get_position();

    assert!(dot(&ab, &cm.contact_infos.normal_a_to_b) > 0.0);

    let r = collision_world.are_colliding(plane_id, obb_id);
    assert!(r.is_some());

    let cm = r.unwrap();
    let a = collision_world
        .collision_object_ref(cm.id_collision_object_a)
        .unwrap();
    let b = collision_world
        .collision_object_ref(cm.id_collision_object_b)
        .unwrap();
    let ab = b.shape.get_position() - a.shape.get_position();

    assert!(dot(&ab, &cm.contact_infos.normal_a_to_b) > 0.0);
}


fn impulsion_and_normal() {
    // detection (obb, plane)
    {
        // Simulation related
        let mut simulation_world = SimulationWorld::new(None);

        // add a cube
        let half_length = Vec3::new(1.0, 1.0, 1.0);
        let obb_co = CollisionObject::new(Box::new(shapes::OBB::new(half_length)));
        let obb_rb = RigidBody::new(
            Transform::translation(P3::new(0.0, -9.049723, 0.0)),
            1.0,
            false,
        );
        let obb_id = simulation_world.add_rigidbody(obb_rb, obb_co);

        // add a plane
        let plane_co = CollisionObject::new(Box::new(shapes::Plane::new(Directions::up())));
        let plane_rb = RigidBody::new(Transform::translation(P3::new(0.0, -10.0, 0.0)), 1.0, true);
        let plane_id = simulation_world.add_rigidbody(plane_rb, plane_co);

        // on assert que la premiere detection de collision existe
        // mais on ne l'utilise pas
        let r = simulation_world
            .collision_world
            .are_colliding(obb_id, plane_id);
        assert!(r.is_some());

        let obb_pos_t = simulation_world.rigidbody_ref(obb_id).transform.translation;
        let plane2obb_tone = obb_pos_t
            - simulation_world
                .rigidbody_ref(plane_id)
                .transform
                .translation;

        // on fait avance manuellement la simulation en choisissant l'ordre dans la collision
        // jusqu'a ce qu'il n'y est plus de contact
        loop {
            if let Some(cm) = simulation_world
                .collision_world
                .are_colliding(obb_id, plane_id)
            {
                simulation_world.collision_world.contact_manifolds.push(cm);
                simulation_world.solve_contact_manifolds();
                simulation_world.collision_world.clear_manifold();

                // on fait avancer le obb a l'etat suivant
                simulation_world.step_rigidbody(obb_id);
                simulation_world.step_rigidbody(plane_id);
            } else {
                break;
            }
        }

        let plane2obb_ttwo = simulation_world.rigidbody_ref(obb_id).transform.translation
            - simulation_world
                .rigidbody_ref(plane_id)
                .transform
                .translation;
        println!(
            "pos avant={:?}  pos apres={:?}",
            obb_pos_t,
            simulation_world.rigidbody_ref(obb_id).transform.translation
        );

        assert!(magnitude(&plane2obb_ttwo) > magnitude(&plane2obb_tone));
        assert!(dot(&normalized(&plane2obb_ttwo), &plane2obb_tone) > 0.0);
    }

    // la seule chose qui change est l'ordre de la detection de collision = (plane, obb)
    {
        // Simulation related
        let mut simulation_world = SimulationWorld::new(None);

        // add a cube
        let half_length = Vec3::new(1.0, 1.0, 1.0);
        let obb_co = CollisionObject::new(Box::new(shapes::OBB::new(half_length)));
        let obb_rb = RigidBody::new(
            Transform::translation(P3::new(0.0, -9.049723, 0.0)),
            1.0,
            false,
        );
        let obb_id = simulation_world.add_rigidbody(obb_rb, obb_co);

        // add a plane
        let plane_co = CollisionObject::new(Box::new(shapes::Plane::new(Directions::up())));
        let plane_rb = RigidBody::new(Transform::translation(P3::new(0.0, -10.0, 0.0)), 1.0, true);
        let plane_id = simulation_world.add_rigidbody(plane_rb, plane_co);

        // on assert que la premiere detection de collision existe
        // mais on ne l'utilise pas
        let r = simulation_world
            .collision_world
            .are_colliding(plane_id, obb_id);
        assert!(r.is_some());

        let obb_pos_t = simulation_world.rigidbody_ref(obb_id).transform.translation;
        let plane2obb_tone = obb_pos_t
            - simulation_world
                .rigidbody_ref(plane_id)
                .transform
                .translation;

        // on fait avance manuellement la simulation en choisissant l'ordre dans la collision
        // jusqu'a ce qu'il n'y est plus de contact
        loop {
            if let Some(cm) = simulation_world
                .collision_world
                .are_colliding(plane_id, obb_id)
            {
                simulation_world.collision_world.contact_manifolds.push(cm);
                simulation_world.solve_contact_manifolds();
                simulation_world.collision_world.clear_manifold();

                // on fait avancer le obb a l'etat suivant
                simulation_world.step_rigidbody(obb_id);
                simulation_world.step_rigidbody(plane_id);
            } else {
                break;
            }
        }

        let plane2obb_ttwo = simulation_world.rigidbody_ref(obb_id).transform.translation
            - simulation_world
                .rigidbody_ref(plane_id)
                .transform
                .translation;
        println!(
            "pos avant={:?}  pos apres={:?}",
            obb_pos_t,
            simulation_world.rigidbody_ref(obb_id).transform.translation
        );

        assert!(magnitude(&plane2obb_ttwo) > magnitude(&plane2obb_tone));
        assert!(dot(&normalized(&plane2obb_ttwo), &plane2obb_tone) > 0.0);
    }
}
