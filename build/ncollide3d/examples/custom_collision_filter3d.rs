//extern crate nalgebra as na;
//extern crate ncollide3d;
//
//use ncollide3d::broad_phase::BroadPhasePairFilter;
//use ncollide3d::shape::{Ball, ShapeHandle};
//use ncollide3d::pipeline::{CollisionGroups, CollisionObject, CollisionObjectSlabHandle, CollisionWorld, GeometricQueryType};
//
//struct ParityFilter;
//
//impl BroadPhasePairFilter<f32, CollisionObject<f32, ()>, CollisionObjectSlabHandle> for ParityFilter {
//    fn is_pair_valid(&self, _: &CollisionObject<f32, ()>, _: &CollisionObject<f32, ()>, handle1: CollisionObjectSlabHandle, handle2: CollisionObjectSlabHandle) -> bool {
//        handle1.uid() % 2 == handle2.uid() % 2
//    }
//}

fn main() {
//    let shape = ShapeHandle::new_shared(Ball::new(0.5f32));
//    let groups = CollisionGroups::new();
//    let query = GeometricQueryType::Contacts(0.0, 0.0);
//
//    let mut world = CollisionWorld::new(0.02);
//
//    world.register_broad_phase_pair_filter("Parity filter", ParityFilter);
//
//    world.add(na::one(), shape.clone(), groups, query, ());
//    world.add(na::one(), shape.clone(), groups, query, ());
//    world.add(na::one(), shape.clone(), groups, query, ());
//    world.add(na::one(), shape.clone(), groups, query, ());
//
//    world.update();
//
//    // There will be only 2 contacts instead of 6.
//    assert!(world.contact_pairs(true).count() == 2);
}
