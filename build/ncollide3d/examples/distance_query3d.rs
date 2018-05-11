#[macro_use]
extern crate approx; // for relative_eq!
extern crate nalgebra as na;
extern crate ncollide3d;

use na::{Isometry3, Vector3};
use ncollide3d::shape::{Ball, Cuboid};
use ncollide3d::query;

fn main() {
    let cuboid = Cuboid::new(Vector3::new(1.0, 1.0, 1.0));
    let ball = Ball::new(1.0);

    let cuboid_pos = na::one();
    let ball_pos_intersecting = Isometry3::new(Vector3::new(1.0, 0.0, 0.0), na::zero());
    let ball_pos_disjoint = Isometry3::new(Vector3::new(3.0, 0.0, 0.0), na::zero());

    let dist_intersecting = query::distance(&ball_pos_intersecting, &ball, &cuboid_pos, &cuboid);
    let dist_disjoint = query::distance(&ball_pos_disjoint, &ball, &cuboid_pos, &cuboid);

    assert_eq!(dist_intersecting, 0.0);
    assert!(relative_eq!(dist_disjoint, 1.0, epsilon = 1.0e-7));
}
