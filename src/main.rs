// https://github.com/sebcrozet/kiss3d 

extern crate kiss3d;
// extern crate nalgebra as na;

use kiss3d::light::Light;
use kiss3d::nalgebra::{UnitQuaternion, Vector3};
use kiss3d::window::Window;

fn main() {
    let mut window = Window::new("Kiss3d: cube");
    let mut c = window.add_cube(1.0, 1.0, 1.0);

    c.set_color(1.0, 0.0, 0.0);

    window.set_light(Light::StickToCamera);
    let rot = UnitQuaternion::<f32>::from_axis_angle(&Vector3::y_axis(), 0.014);

    while window.render() {
        
    }
}
