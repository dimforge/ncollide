extern crate nalgebra as na;

use na::{Isometry2, Vector2};
use ncollide2d::bounding_volume::{self, BoundingVolume};
use ncollide2d::shape::Ball;

fn main() {
    /*
     * Initialize the shapes.
     */
    let ball1 = Ball::new(0.5);
    let ball2 = Ball::new(1.0);

    let ball1_pos = Isometry2::new(Vector2::y(), na::zero()); // 1.0 along the `y` axis.
    let ball2_pos = Isometry2::identity(); // Identity matrix.

    /*
     * Compute their axis-aligned bounding boxes.
     */
    let aabb_ball1 = bounding_volume::aabb(&ball1, &ball1_pos);
    let aabb_ball2 = bounding_volume::aabb(&ball2, &ball2_pos);

    // Merge the two boxes.
    let bounding_aabb = aabb_ball1.merged(&aabb_ball2);

    // Enlarge the ball2 aabb.
    let loose_aabb_ball2 = aabb_ball2.loosened(1.0);

    // Intersection and inclusion tests.
    assert!(aabb_ball1.intersects(&aabb_ball2));
    assert!(bounding_aabb.contains(&aabb_ball1));
    assert!(bounding_aabb.contains(&aabb_ball2));
    assert!(!aabb_ball2.contains(&bounding_aabb));
    assert!(!aabb_ball1.contains(&bounding_aabb));
    assert!(loose_aabb_ball2.contains(&aabb_ball2));
}
