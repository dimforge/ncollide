extern crate "nalgebra" as na;
extern crate "ncollide2df32" as ncollide;

use na::Vec2;
use ncollide::geom::Plane;

fn main() {
    let plane = Plane::new(Vec2::new(0.0, 1.0));

    assert!(plane.normal().x == 0.0);
    assert!(plane.normal().y == 1.0);
}
