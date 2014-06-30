extern crate ncollide = "ncollide2df32";

use ncollide::geom::Ball;

fn main() {
    let ball = Ball::new(1.0);
    assert!(ball.radius() == 1.0);
}
