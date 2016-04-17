extern crate nalgebra as na;
extern crate ncollide;

use std::sync::Arc;
use na::{Isometry3, Vector3};
use ncollide::inspection::Repr3;
use ncollide::shape::{Cuboid, Compound};

fn main() {
    // Delta transformation matrices.
    let delta1 = Isometry3::new(Vector3::new(0.0f32, -1.5, 0.0), na::zero());
    let delta2 = Isometry3::new(Vector3::new(-1.5f32, 0.0, 0.0), na::zero());
    let delta3 = Isometry3::new(Vector3::new(1.5f32, 0.0,  0.0), na::zero());

    // 1) Initialize the shape list.
    let mut shapes = Vec::new();
    let horizontal_box = Arc::new(Box::new(Cuboid::new(Vector3::new(1.5f32,  0.25, 0.25))) as Box<Repr3<f32>>);
    let vertical_box   = Arc::new(Box::new(Cuboid::new(Vector3::new(0.25f32, 1.5, 0.25))) as Box<Repr3<f32>>);

    shapes.push((delta1, horizontal_box));
    shapes.push((delta2, vertical_box.clone()));
    shapes.push((delta3, vertical_box));

    // 2) Create the compound shape.
    let compound = Compound::new(shapes);

    assert!(compound.shapes().len() == 3)
}
