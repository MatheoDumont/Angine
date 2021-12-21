extern crate engine;
use engine::collision_detection::{CollisionWorld, collisions::CollisionObject};
use engine::math::*;

#[test]
fn basic() {
	let world = CollisionWorld::new();
	let sphere = 
	world.add_collision_object(CollisionObject::new(Point::origin(), shape: Box<dyn Shape>))
}
