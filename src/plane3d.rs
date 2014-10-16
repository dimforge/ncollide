extern crate "nalgebra" as na;
extern crate "ncollide3df32" as ncollide;

use na::Vec3;
use ncollide::geom::Plane;

fn main() {
    let plane = Plane::new(Vec3::new(0.0, 1.0, 0.0));

    assert!(plane.normal().x == 0.0);
    assert!(plane.normal().y == 1.0);
    assert!(plane.normal().z == 0.0);
}
