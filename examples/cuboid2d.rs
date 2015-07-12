extern crate nalgebra as na;
extern crate ncollide;

use na::Vec2;
use ncollide::shape::Cuboid;

fn main() {
    let cuboid = Cuboid::new(Vec2::new(2.0f32, 1.0));

    assert!(cuboid.half_extents().x == 2.0);
    assert!(cuboid.half_extents().y == 1.0);
}
