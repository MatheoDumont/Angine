// https://github.com/sebcrozet/kiss3d

extern crate angine;
extern crate kiss3d;

use angine::engine::{
    collision::collision_object::CollisionObject, shapes, simulation::RigidBody,
    simulation::SimulationWorld,
};
use angine::math::math_essentials::*;
use angine::math::Quaternion as AngineQuat;

use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::nalgebra::{Quaternion, Translation3, UnitQuaternion};
use kiss3d::window::Window;

pub fn update_state(
    node: &mut kiss3d::scene::SceneNode,
    angine_object: &RigidBody,
    base_rotation: UnitQuaternion<f32>,
) {
    let transform = angine_object.transform;
    let q = AngineQuat::from_mat3(&transform.rotation);
    let qq = Quaternion::new(q.x, q.y, q.z, q.w);
    let unit_q = UnitQuaternion::from_quaternion(qq);
    node.set_local_rotation(unit_q * base_rotation);
    let t = Translation3::new(
        transform.translation.x(),
        transform.translation.y(),
        transform.translation.z(),
    );
    node.set_local_translation(t);
}

fn main() {
    // Simulation related
    let mut simulation_world = SimulationWorld::new(None);
    // add a cube
    let half_length = Vec3::new(1.0, 1.0, 1.0);
    let obb_co = CollisionObject::new(Box::new(shapes::OBB::new(half_length.clone())));
    let obb_rb = RigidBody::new(Transform::translation(P3::new(0.0, -9.0, 0.0)), 1.0, false);
    let obb_id = simulation_world.add_rigidbody(obb_rb, obb_co);
    // add a plane
    let plane_co = CollisionObject::new(Box::new(shapes::Plane::new(Directions::up())));
    let plane_rb = RigidBody::new(Transform::translation(P3::new(0.0, -10.0, 0.0)), 1.0, true);
    let plane_id = simulation_world.add_rigidbody(plane_rb, plane_co);

    // create window
    let mut window = Window::new("Kiss3d: cube");
    // cube in scene
    let mut cube = window.add_cube(
        2.0 * half_length.x(),
        2.0 * half_length.y(),
        2.0 * half_length.z(),
    );
    cube.set_color(1.0, 0.0, 0.0);

    // plane in scene
    let mut plane = window.add_quad(10.0, 10.0, 1, 1);
    let unit_q = UnitQuaternion::from_axis_angle(
        &kiss3d::nalgebra::Vector3::x_axis(),
        std::f32::consts::FRAC_PI_2,
    );
    plane.append_rotation(&unit_q);
    plane.set_color(0.0, 1.0, 0.0);
    update_state(
        &mut cube,
        simulation_world.rigidbody_ref(obb_id),
        UnitQuaternion::<f32>::identity(),
    );
    update_state(&mut plane, simulation_world.rigidbody_ref(plane_id), unit_q);

    window.set_light(Light::StickToCamera);
    let mut pause = true;
    let mut step = || {
        let t_start = std::time::Instant::now();
        simulation_world.discrete_step();
        update_state(
            &mut cube,
            simulation_world.rigidbody_ref(obb_id),
            UnitQuaternion::<f32>::identity(),
        );
        // even if plane is static in angine
        update_state(&mut plane, simulation_world.rigidbody_ref(plane_id), unit_q);
        let elapsed = t_start.elapsed();
        let i = std::cmp::min(16 as u32, elapsed.as_millis() as u32);
        let i = std::cmp::max(i, 0 as u32);
        std::thread::sleep_ms(16 - i);
    };
    loop {
        if !pause {
            step();
        }

        if !window.render() {
            break;
        }

        for e in window.events().iter() {
            match e.value {
                WindowEvent::Key(p, Action::Release, _) => match p {
                    Key::P => {
                        pause = !pause;
                    }
                    Key::N => {
                        step();
                    }
                    Key::R => {}
                    _ => {}
                },
                _ => {}
            }
        }
    }
}
