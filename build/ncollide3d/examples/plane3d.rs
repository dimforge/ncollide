extern crate nalgebra as na;

use na::Vector3;
use ncollide3d::shape::Plane;

fn main() {
    let plane = Plane::new(Vector3::<f32>::y_axis());

    assert!(plane.normal.x == 0.0);
    assert!(plane.normal.y == 1.0);
    assert!(plane.normal.z == 0.0);
}
