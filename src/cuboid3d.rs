extern crate "nalgebra" as na;
extern crate "ncollide3df32" as ncollide;

use na::Vec3;
use ncollide::geom::Cuboid;

fn main() {
    let cuboid = Cuboid::new(Vec3::new(2.0, 1.0, 3.0));

    assert!(cuboid.half_extents().x == 2.0);
    assert!(cuboid.half_extents().y == 1.0);
    assert!(cuboid.half_extents().z == 3.0);
}
