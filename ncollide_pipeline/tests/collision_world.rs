extern crate ncollide_pipeline;
extern crate ncollide_entities;
extern crate nalgebra;

use ncollide_pipeline::world::{CollisionGroups, CollisionWorld, CollisionWorld2};
use ncollide_entities::shape::Ball;
use ncollide_entities::inspection::Repr2;
use nalgebra::{Vec1, Vector2, Isometry2};
use std::sync::Arc;

type ObjectCollisionWorld = CollisionWorld2<f64, ()>;
type ShapeRepr = Repr2<f64>;


#[test]
fn issue_57_object_remove() {
	let mut world: ObjectCollisionWorld = CollisionWorld::new(0.1, 0.1, false);
    let shape = Arc::new(Box::new(Ball::new(1.0)) as Box<ShapeRepr>);
    world.add(0,
              Isometry2::new(Vector2::new(1.0, 0.0), Vec1::new(0.0)),
              shape.clone(),
              CollisionGroups::new(),
              ());
   world.add(1,
              Isometry2::new(Vector2::new(1.0, 1.0), Vec1::new(0.0)),
              shape.clone(),
              CollisionGroups::new(),
              ());
	world.add(2,
              Isometry2::new(Vector2::new(1.0, 2.0), Vec1::new(0.0)),
              shape.clone(),
              CollisionGroups::new(),
              ());
    world.update();
    world.remove(0);
    world.update();
    world.update();
}
