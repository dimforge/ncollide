extern crate "nalgebra" as na;
extern crate ncollide;

use std::sync::Arc;
use na::Pnt2;
use ncollide::shape::{Mesh, Mesh2};

fn main() {
    let points = vec!(
        Pnt2::new(0.0, 1.0),  Pnt2::new(-1.0, -1.0),
        Pnt2::new(0.0, -0.5), Pnt2::new(1.0, -1.0));

    let indices = vec!(0u, 1,
                       1,  2,
                       2,  3,
                       3,  1);

    // Build the mesh.
    let mesh: Mesh2<f32> = Mesh::new(Arc::new(points), Arc::new(indices), None, None);

    assert!(mesh.vertices().len() == 4);
}
