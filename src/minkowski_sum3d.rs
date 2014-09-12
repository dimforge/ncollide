extern crate nalgebra;
extern crate "ncollide3df32" as ncollide;

use nalgebra::na;
use ncollide::geom::{Cone, Cylinder, MinkowskiSum};

fn main() {
    let cylinder = Cylinder::new(0.5, 0.75);
    let cone     = Cone::new(0.75, 0.75);

    let delta_cylinder = na::one(); // identity matrix.
    let delta_cone     = na::one(); // identity matrix.

    let _ = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &cone);
}
