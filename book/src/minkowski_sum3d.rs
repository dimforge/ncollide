extern crate nalgebra;
extern crate ncollide = "ncollide3df32";

use nalgebra::na;
use ncollide::implicit::HasMargin;
use ncollide::geom::{Cone, Cylinder, MinkowskiSum};

fn main() {
    let cylinder = Cylinder::new(0.5, 0.75);
    let cone     = Cone::new_with_margin(0.75, 0.75, 0.1);

    let delta_cylinder = na::one(); // identity matrix.
    let delta_cone     = na::one(); // identity matrix.

    // Build the Minkowski sum. It will have a total margin of 0.04 + 0.1.
    let sum = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &cone);

    assert!(sum.margin() == 0.14);
}
