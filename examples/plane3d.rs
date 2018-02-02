extern crate nalgebra as na;
extern crate ncollide;

use na::Vector3;
use ncollide::shape::Plane;

fn main() {
    let plane = Plane::new(Vector3::<f32>::y_axis());

    assert!(plane.normal().as_ref().x == 0.0);
    assert!(plane.normal().as_ref().y == 1.0);
    assert!(plane.normal().as_ref().z == 0.0);
}
