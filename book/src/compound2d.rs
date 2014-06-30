extern crate nalgebra;
extern crate ncollide = "ncollide2df32";

use nalgebra::na::{Iso2, Vec2};
use nalgebra::na;
use ncollide::geom::{Cuboid, Compound, CompoundData};

fn main() {
    // delta transformation matrices.
    let delta1 = Iso2::new(Vec2::new(0.0f32, -1.5), na::zero());
    let delta2 = Iso2::new(Vec2::new(-1.5f32, 0.0), na::zero());
    let delta3 = Iso2::new(Vec2::new(1.5f32,  0.0), na::zero());

    // 1) initialize the CompoundData.
    let mut compound_data = CompoundData::new();
    compound_data.push_geom(delta1, Cuboid::new(Vec2::new(1.5f32, 0.75)), 1.0);
    compound_data.push_geom(delta2, Cuboid::new(Vec2::new(0.75f32, 1.5)), 1.0);
    compound_data.push_geom(delta3, Cuboid::new(Vec2::new(0.75f32, 1.5)), 1.0);

    // 2) create the compound geometry.
    let compound = Compound::new(compound_data);

    assert!(compound.geoms().len() == 3)
}
