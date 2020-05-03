extern crate nalgebra as na;

use na::{Isometry2, Vector2};
use ncollide2d::shape::{Compound, Cuboid, ShapeHandle};

fn main() {
    // Delta transformation matrices.
    let delta1 = Isometry2::new(Vector2::new(0.0f32, -1.5), na::zero());
    let delta2 = Isometry2::new(Vector2::new(-1.5f32, 0.0), na::zero());
    let delta3 = Isometry2::new(Vector2::new(1.5f32, 0.0), na::zero());

    // 1) Initialize the shape list.
    let mut shapes = Vec::new();
    let horizontal_box = ShapeHandle::new(Cuboid::new(Vector2::new(1.5f32, 0.25)));
    let vertical_box = ShapeHandle::new(Cuboid::new(Vector2::new(0.25f32, 1.5)));

    shapes.push((delta1, horizontal_box));
    shapes.push((delta2, vertical_box.clone()));
    shapes.push((delta3, vertical_box));

    // 2) Create the compound shape.
    let compound = Compound::new(shapes);

    assert!(compound.shapes().len() == 3)
}
