extern crate nalgebra as na;
extern crate ncollide;

use na::Isometry2;
use ncollide::shape::{Cone, Cylinder, Reflection, MinkowskiSum};

fn main() {
    let cylinder   = Cylinder::new(0.5f32, 0.75);
    let cone       = Cone::new(0.75f32, 0.75);
    let reflection = Reflection::new(&cone); // Take the reflection of the cone.

    let delta_cylinder = na::one::<Isometry2<f32>>(); // Id matrix.
    let delta_cone     = na::one::<Isometry2<f32>>(); // Id matrix.

    // Build the Configuration Space Obstacle.
    let _ = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &reflection);
}
