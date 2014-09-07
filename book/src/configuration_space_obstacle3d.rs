extern crate nalgebra;
extern crate "ncollide3df32" as ncollide;

use nalgebra::na;
use ncollide::implicit::HasMargin;
use ncollide::geom::{Cone, Cylinder, Reflection, MinkowskiSum};

fn main() {
    let cylinder   = Cylinder::new(0.5, 0.75);
    let cone       = Cone::new_with_margin(0.75, 0.75, 0.1);
    let reflection = Reflection::new(&cone); // take the reflection of the cone.

    let delta_cylinder = na::one(); // identity matrix.
    let delta_cone     = na::one(); // identity matrix.

    // Build the Configuration Space Obstacle.
    let cso = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &reflection);

    assert!(cso.margin() == 0.14);
}
