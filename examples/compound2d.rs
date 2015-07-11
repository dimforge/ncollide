extern crate nalgebra as na;
extern crate ncollide;

use std::sync::Arc;
use na::{Iso2, Vec2};
use ncollide::shape::{Cuboid, Compound};
use ncollide::inspection::Repr2;

fn main() {
    // Delta transformation matrices.
    let delta1 = Iso2::new(Vec2::new(0.0f32, -1.5), na::zero());
    let delta2 = Iso2::new(Vec2::new(-1.5f32, 0.0), na::zero());
    let delta3 = Iso2::new(Vec2::new(1.5f32,  0.0), na::zero());

    // 1) Initialize the shape list.
    let mut shapes = Vec::new();
    let horizontal_box = Arc::new(Box::new(Cuboid::new(Vec2::new(1.5f32,  0.25))) as Box<Repr2<f32>>);
    let vertical_box   = Arc::new(Box::new(Cuboid::new(Vec2::new(0.25f32, 1.5))) as Box<Repr2<f32>>);

    shapes.push((delta1, horizontal_box));
    shapes.push((delta2, vertical_box.clone()));
    shapes.push((delta3, vertical_box));

    // 2) Create the compound shape.
    let compound = Compound::new(shapes);

    assert!(compound.shapes().len() == 3)
}
