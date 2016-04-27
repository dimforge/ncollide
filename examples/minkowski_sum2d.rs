extern crate nalgebra as na;
extern crate ncollide;

use na::Isometry2;
use ncollide::shape::{Cone, Cylinder, MinkowskiSum};

fn main() {
    let cylinder = Cylinder::new(0.5f32, 0.75);
    let cone     = Cone::new(0.75f32, 0.75);

    let delta_cylinder = na::one::<Isometry2<f32>>(); // Identity matrix.
    let delta_cone     = na::one::<Isometry2<f32>>(); // Identity matrix.

    let _ = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &cone);
}
