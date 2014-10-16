extern crate "nalgebra" as na;
extern crate "ncollide3df32" as ncollide;

use ncollide::geom::{Cone, Cylinder, Reflection, MinkowskiSum};

fn main() {
    let cylinder   = Cylinder::new(0.5, 0.75);
    let cone       = Cone::new(0.75, 0.75);
    let reflection = Reflection::new(&cone); // Take the reflection of the cone.

    let delta_cylinder = na::one(); // identity matrix.
    let delta_cone     = na::one(); // identity matrix.

    // Build the Configuration Space Obstacle.
    let _ = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &reflection);
}
