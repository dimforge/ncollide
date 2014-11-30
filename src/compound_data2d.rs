extern crate "nalgebra" as na;
extern crate ncollide;

use std::sync::Arc;
use na::{Iso2, Vec2};
use ncollide::shape::{Shape2, Plane, Cuboid, Compound, CompoundData};
use ncollide::volumetric::Volumetric;

fn main() {
    // delta transformation matrices.
    let delta1 = Iso2::new(Vec2::new(0.0, -1.5), na::zero());
    let delta2 = Iso2::new(Vec2::new(-1.5, 0.0), na::zero());
    let delta3 = Iso2::new(Vec2::new(1.5,  0.0), na::zero());

    // 1) initialize the CompoundData.
    let mut compound_data = CompoundData::new();

    /*
     * push_shape
     */
    // The mass, center of mass and angular inertia tensor are automatically
    // computed.
    compound_data.push_shape(delta1, Cuboid::new(Vec2::new(1.5, 0.75)), 1.0);

    /*
     * push_shape_with_mass_properties
     */
    // area                   = 1.0
    // mass                   = 10.0
    // center of mass         = the origin (na::zero())
    // angular inertia tensor = identity matrix (na::one())
    compound_data.push_shape_with_mass_properties(
        delta2,
        Plane::new(Vec2::new(1.0, 0.0)),
        (na::one(), 10.0, na::orig(), na::one()));

    /*
     * push_shared_shape_with_mass_properties
     */
    // The shape we want to share.
    let cuboid = Cuboid::new(Vec2::new(0.75, 1.5));
    // Make ncollide compute the mass properties of the cuboid.
    let (c_mass, c_com, c_tensor) = cuboid.mass_properties(1.0); // density = 1.0
    let c_area                    = cuboid.surface();
    // Build the shared shape.
    let shared_cuboid = Arc::new(box cuboid as Box<Shape2<f32>>);
    // Add the shape to the compound data.
    compound_data.push_shared_shape_with_mass_properties(
        delta3,
        shared_cuboid.clone(),
        (c_area, c_mass, c_com, c_tensor));
    // `shared_cuboid` can still be used thereafter…

    // 2) create the compound shape.
    let compound = Compound::new(compound_data);

    assert!(compound.shapes().len() == 3);
}
