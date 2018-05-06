extern crate nalgebra;
extern crate ncollide2d;

use ncollide2d::world::{CollisionGroups, CollisionWorld};
use ncollide2d::shape::{Ball, ShapeHandle};
use ncollide2d::world::GeometricQueryType;
use nalgebra::{Isometry2, Vector2};

#[test]
fn issue_57_object_remove() {
    let mut world = CollisionWorld::new(0.1);
    let shape = ShapeHandle::new(Ball::new(1.0));
    let contact_query = GeometricQueryType::Contacts(0.0, 0.0);
    let object1 = world.add(
        Isometry2::new(Vector2::new(1.0, 0.0), 0.0),
        shape.clone(),
        CollisionGroups::new(),
        contact_query,
        (),
    );
    let _ = world.add(
        Isometry2::new(Vector2::new(1.0, 1.0), 0.0),
        shape.clone(),
        CollisionGroups::new(),
        contact_query,
        (),
    );
    let _ = world.add(
        Isometry2::new(Vector2::new(1.0, 2.0), 0.0),
        shape.clone(),
        CollisionGroups::new(),
        contact_query,
        (),
    );
    world.update();
    world.remove(&[object1]);
    world.update();
    world.update();
}
