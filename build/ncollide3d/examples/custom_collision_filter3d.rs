extern crate nalgebra as na;
extern crate ncollide3d;

use ncollide3d::shape::{Ball, ShapeHandle};
use ncollide3d::broad_phase::BroadPhasePairFilter;
use ncollide3d::world::{CollisionObject, CollisionWorld, GeometricQueryType};

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
    let query = GeometricQueryType::Contacts(0.0, 0.0);

    let mut world = CollisionWorld::new(0.02);

    world.register_broad_phase_pair_filter("Parity filter", ParityFilter);

    world.add(na::one(), shape.clone(), 0, query, ());
    world.add(na::one(), shape.clone(), 0, query, ());
    world.add(na::one(), shape.clone(), 0, query, ());
    world.add(na::one(), shape.clone(), 0, query, ());

    world.update();

    // There will be only 2 contacts instead of 6.
    assert!(world.contact_manifolds().count() == 2);
}
