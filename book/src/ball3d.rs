extern crate ncollide = "ncollide3df32";

use ncollide::geom::Ball;

fn main() {
    let ball = Ball::new(1.0);
    assert!(ball.radius() == 1.0);
}
