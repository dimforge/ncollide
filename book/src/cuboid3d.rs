extern crate nalgebra;
extern crate "ncollide3df32" as ncollide;

use nalgebra::na::Vec3;
use ncollide::geom::Cuboid;

fn main() {
    let cuboid = Cuboid::new_with_margin(Vec3::new(2.0, 1.0, 3.0), 0.2);
    assert!(cuboid.margin() == 0.2); // user-defined margin
}
