extern crate ncollide3d;

use ncollide3d::shape::Ball;

fn main() {
    let ball = Ball::new(1.0f32);
    assert!(ball.radius() == 1.0);
}
