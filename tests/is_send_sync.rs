// NOTE: the following tests will compile iff. the tested structs are `Send + Sync + 'static`.
extern crate ncollide;

use ncollide::world::CollisionWorld3;

fn is_send_sync<T: Send + Sync + 'static>(_: T) -> bool {
    true
}

#[test]
fn world_is_send_sync() {
    let world = CollisionWorld3::<f32, ()>::new(0.0f32);
    assert!(is_send_sync(world));
}
