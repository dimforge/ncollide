extern crate nalgebra as na;
extern crate ncollide;

use na::{Vector2, Isometry2};
use ncollide::shape::Ball;
use ncollide::bounding_volume;
use ncollide::broad_phase::{DBVTBroadPhase, BroadPhase};

fn main() {
    /*
     * Create the objects.
     */
    let poss = [ Isometry2::new(Vector2::new(0.0, 0.0), na::zero()),
                 Isometry2::new(Vector2::new(0.0, 0.5), na::zero()),
                 Isometry2::new(Vector2::new(0.5, 0.0), na::zero()),
                 Isometry2::new(Vector2::new(0.5, 0.5), na::zero()) ];

    // We will use the same geometry for the four objects.
    let ball = Ball::new(0.5);

    /*
     * Create the broad phase.
     * We know we will use small uids (from 0 to 3)so we can pass `true` as the second argument.
     */
    let mut bf = DBVTBroadPhase::new(0.2, true);

    // First parameter:  a unique id for each object.
    // Second parameter: the object bounding box.
    // Third parameter:  some data (here, the id that identify each object).
    bf.deferred_add(0, bounding_volume::aabb(&ball, &poss[0]), 0);
    bf.deferred_add(1, bounding_volume::aabb(&ball, &poss[1]), 1);
    bf.deferred_add(2, bounding_volume::aabb(&ball, &poss[2]), 2);
    bf.deferred_add(3, bounding_volume::aabb(&ball, &poss[3]), 3);

    // Update the broad phase.
    // The collision filter (first closure) prevents self-collision.
    bf.update(&mut |a, b| *a != *b, &mut |_, _, _| { });

    assert!(bf.num_interferences() == 6);

    // Remove two objects.
    bf.deferred_remove(0);
    bf.deferred_remove(1);

    // Update the broad phase.
    // The collision filter (first closure) prevents self-collision.
    bf.update(&mut |a ,b| *a != *b, &mut |_, _, _| { });

    assert!(bf.num_interferences() == 1)
}
