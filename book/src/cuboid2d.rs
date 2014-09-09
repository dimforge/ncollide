extern crate nalgebra;
extern crate "ncollide2df32" as ncollide;

use nalgebra::na::Vec2;
use ncollide::geom::Cuboid;

fn main() {
    let cuboid = Cuboid::new(Vec2::new(2.0, 1.0));

    assert!(cuboid.half_extents().x == 2.0);
    assert!(cuboid.half_extents().y == 1.0);
}
