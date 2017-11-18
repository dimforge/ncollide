extern crate ncollide_pipeline;
extern crate ncollide_geometry;
extern crate nalgebra;

use ncollide_pipeline::world::{CollisionGroups, CollisionWorld, GeometricQueryType};
use ncollide_geometry::shape::{Ball, ShapeHandle};
use nalgebra as na;

#[test]
fn issue_57_object_remove() {
    let mut world: CollisionWorld<_,_,_> = CollisionWorld::new(0.1, false);
    let shape = Ball::new(1.0);

    world.deferred_add(
        0,
        na::Isometry2::new(na::Vector2::new(1.0, 0.0), 0.0),
        ShapeHandle::new(shape.clone()),
        CollisionGroups::new(),
        GeometricQueryType::Contacts(0.0),
        ());
    world.deferred_add(
        1,
        na::Isometry2::new(na::Vector2::new(1.0, 0.0), 0.0),
        ShapeHandle::new(shape.clone()),
        CollisionGroups::new(),
        GeometricQueryType::Contacts(0.0),
        ());
    world.perform_additions_removals_and_broad_phase();
    world.deferred_remove(0);
    world.perform_additions_removals_and_broad_phase();
}
