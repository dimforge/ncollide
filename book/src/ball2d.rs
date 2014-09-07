extern crate "ncollide2df32" as ncollide;

use ncollide::geom::Ball;

fn main() {
    let ball = Ball::new(1.0);
    assert!(ball.radius() == 1.0);
}
