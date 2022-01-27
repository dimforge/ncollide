extern crate nalgebra as na;

use na::Vector2;
use ncollide2d::shape::Plane;

fn main() {
    let plane = Plane::new(Vector2::<f32>::y_axis());

    assert!(plane.normal.x == 0.0);
    assert!(plane.normal.y == 1.0);
}
