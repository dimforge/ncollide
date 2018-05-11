extern crate nalgebra as na;
extern crate ncollide2d;

use ncollide2d::shape::{Ball, ShapeHandle};
use ncollide2d::broad_phase::BroadPhasePairFilter;
use ncollide2d::world::{CollisionGroups, CollisionObject, CollisionWorld, GeometricQueryType};

struct ParityFilter;

impl BroadPhasePairFilter<f32, ()> for ParityFilter {
    fn is_pair_valid(
        &self,
        b1: &CollisionObject<f32, ()>,
        b2: &CollisionObject<f32, ()>,
    ) -> bool {
        b1.handle().uid() % 2 == b2.handle().uid() % 2
    }
}

fn main() {
    let shape = ShapeHandle::new(Ball::new(0.5f32));
    let groups = CollisionGroups::new();
    let query = GeometricQueryType::Contacts(0.0, 0.0);

    let mut world = CollisionWorld::new(0.02);

    world.register_broad_phase_pair_filter("Parity filter", ParityFilter);

    world.add(na::one(), shape.clone(), groups, query, ());
    world.add(na::one(), shape.clone(), groups, query, ());
    world.add(na::one(), shape.clone(), groups, query, ());
    world.add(na::one(), shape.clone(), groups, query, ());

    world.update();

    // There will be only 2 contacts instead of 6.
    assert!(world.contact_manifolds().count() == 2);
}
