extern crate nalgebra as na;

use na::{Isometry2, Vector2};
use ncollide2d::query;
use ncollide2d::shape::{Ball, Cuboid};

fn main() {
    let cuboid = Cuboid::new(Vector2::new(1.0, 1.0));
    let ball = Ball::new(1.0);
    let prediction = 1.0;

    let cuboid_pos = na::one();
    let ball_pos_penetrating = Isometry2::new(Vector2::new(1.0, 1.0), na::zero());
    let ball_pos_in_prediction = Isometry2::new(Vector2::new(2.0, 2.0), na::zero());
    let ball_pos_too_far = Isometry2::new(Vector2::new(3.0, 3.0), na::zero());

    let ctct_penetrating = query::contact(
        &ball_pos_penetrating,
        &ball,
        &cuboid_pos,
        &cuboid,
        prediction,
    );
    let ctct_in_prediction = query::contact(
        &ball_pos_in_prediction,
        &ball,
        &cuboid_pos,
        &cuboid,
        prediction,
    );
    let ctct_too_far = query::contact(&ball_pos_too_far, &ball, &cuboid_pos, &cuboid, prediction);

    assert!(ctct_penetrating.unwrap().depth > 0.0);
    assert!(ctct_in_prediction.unwrap().depth < 0.0);
    assert_eq!(ctct_too_far, None);
}
