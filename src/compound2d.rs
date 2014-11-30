extern crate "nalgebra" as na;
extern crate ncollide;

use na::{Iso2, Vec2};
use ncollide::shape::{Cuboid, Compound, CompoundData};

fn main() {
    // Delta transformation matrices.
    let delta1 = Iso2::new(Vec2::new(0.0f32, -1.5), na::zero());
    let delta2 = Iso2::new(Vec2::new(-1.5f32, 0.0), na::zero());
    let delta3 = Iso2::new(Vec2::new(1.5f32,  0.0), na::zero());

    // 1) Initialize the CompoundData.
    let mut compound_data = CompoundData::new();
    compound_data.push_shape(delta1, Cuboid::new(Vec2::new(1.5, 0.25)), 1.0);
    compound_data.push_shape(delta2, Cuboid::new(Vec2::new(0.25, 1.5)), 1.0);
    compound_data.push_shape(delta3, Cuboid::new(Vec2::new(0.25, 1.5)), 1.0);

    // 2) Create the compound shape.
    let compound = Compound::new(compound_data);

    assert!(compound.shapes().len() == 3)
}
