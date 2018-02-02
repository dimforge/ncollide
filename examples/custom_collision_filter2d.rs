extern crate nalgebra as na;
extern crate ncollide;

use na::{Isometry2, Point2};
use ncollide::shape::{Ball, ShapeHandle};
use ncollide::broad_phase::BroadPhasePairFilter;
use ncollide::world::{CollisionGroups, CollisionObject2, CollisionWorld, GeometricQueryType};

struct ParityFilter;

impl BroadPhasePairFilter<Point2<f32>, Isometry2<f32>, ()> for ParityFilter {
    fn is_pair_valid(
        &self,
        b1: &CollisionObject2<f32, ()>,
        b2: &CollisionObject2<f32, ()>,
    ) -> bool {
        b1.uid % 2 == b2.uid % 2
    }
}

fn main() {
    let shape = ShapeHandle::new(Ball::new(0.5f32));
    let groups = CollisionGroups::new();
    let query = GeometricQueryType::Contacts(0.0);

    let mut world = CollisionWorld::new(0.02, true);

    world.register_broad_phase_pair_filter("Parity filter", ParityFilter);

    world.deferred_add(0, na::one(), shape.clone(), groups, query, ());
    world.deferred_add(1, na::one(), shape.clone(), groups, query, ());
    world.deferred_add(2, na::one(), shape.clone(), groups, query, ());
    world.deferred_add(3, na::one(), shape.clone(), groups, query, ());

    world.update();

    // There will be only 2 contacts instead of 6.
    assert!(world.contacts().count() == 2);
}
