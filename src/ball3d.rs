extern crate ncollide;

use ncollide::shape::Ball;

fn main() {
    let ball = Ball::new(1.0f32);
    assert!(ball.radius() == 1.0);
}
