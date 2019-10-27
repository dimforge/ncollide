use nalgebra::{Isometry3, Translation, Vector3, zero};
use ncollide3d::{
    shape::{Cuboid, ShapeHandle},
    pipeline::{CollisionGroups, CollisionWorld, GeometricQueryType},
};
use std::collections::HashMap;

#[test]
fn two_colliding_cuboids() {
    let mut world = CollisionWorld::new(0.0);

    // Add two intersecting cuboids to the world.
    let mut groups = CollisionGroups::new();
    groups.set_membership(&[1]);
    groups.set_whitelist(&[1]);
    let contacts_query = GeometricQueryType::Contacts(0.0, 0.0);
    let shape = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 1.0, 1.0)));
    let pos = Isometry3::new(zero(), zero());
    world.add(pos, shape.clone(), groups, contacts_query, 1);
    world.add(pos, shape, groups, contacts_query, 2);

    loop {
        // BUG: updating doesn't clear out the old contact pairs, so in the loop below, we will try
        // pushing apart objects that, according to their positions and shapes, should not be
        // in contact anymore.
        world.update();

        // Resolve intersections by pushing cubes apart along the contact normal.
        let mut move_actions = HashMap::new();
        for (handle1, handle2, _, manifold) in world.contact_pairs(true) {
            // Once a room from a contact has been moved, it may invalidate other contacts that
            // it was a part of, so skip them.
            if move_actions.contains_key(&handle1) || move_actions.contains_key(&handle2) {
                continue;
            }

            let contact = &manifold.deepest_contact()
                .expect("No penetration in contact").contact;

            if contact.depth == 0.0 {
                continue;
            }

            // The normal should be parallel to some axis. To ensure that there are no endless
            // cycles of resolutions, only allow resolutions to push in the positive direction
            // along each axis.
            let n: Vector3<f32> = contact.depth * contact.normal.into_inner();
            let (push_v, move_handle) = if n.x > 0.0 || n.y > 0.0 || n.z > 0.0 {
                println!("Pushing room 1");
                (-n, handle1)
            } else {
                println!("Pushing room 2");
                (n, handle2)
            };

            move_actions.insert(move_handle, push_v);

            let obj1 = world.collision_object(handle1).unwrap();
            let obj2 = world.collision_object(handle2).unwrap();
            println!("C1 = {:?}", obj1.shape().as_shape::<Cuboid<f32>>());
            println!("C2 = {:?}", obj2.shape().as_shape::<Cuboid<f32>>());
            println!("C1 = {:?}", obj1.position().translation.vector);
            println!("C2 = {:?}", obj2.position().translation.vector);
            println!("Depth = {}", contact.depth);
            println!("N = {}", contact.normal.into_inner());
        }

        // Need to perform movements after releasing the borrow on world.
        for (handle, push_v) in move_actions.iter() {
            let move_obj = world.objects.get_mut(*handle)
                .expect("Collision object does not exist for handle");
            let mut pos = move_obj.position().clone();
            pos.append_translation_mut(&Translation::from(*push_v));
            move_obj.set_position(pos);
        }

        if move_actions.is_empty() {
            break;
        }
    }
}