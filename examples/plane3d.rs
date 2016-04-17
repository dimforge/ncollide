extern crate nalgebra as na;
extern crate ncollide;

use na::Vector3;
use ncollide::shape::Plane;

fn main() {
    let plane = Plane::new(Vector3::new(0.0f32, 1.0, 0.0));

    assert!(plane.normal().x == 0.0);
    assert!(plane.normal().y == 1.0);
    assert!(plane.normal().z == 0.0);
}
