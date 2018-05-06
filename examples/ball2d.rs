extern crate ncollide2d;

use ncollide2d::shape::Ball;

fn main() {
    let ball = Ball::new(1.0f32);
    assert!(ball.radius() == 1.0);
}
