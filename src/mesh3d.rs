extern crate "nalgebra" as na;
extern crate ncollide;

use std::sync::Arc;
use na::Pnt3;
use ncollide::shape::{Mesh, Mesh3};

fn main() {
    let points = vec!(
        Pnt3::new(0.0f32, 1.0, 0.0), Pnt3::new(-1.0, -0.5, 0.0),
        Pnt3::new(0.0, -0.5, -1.0), Pnt3::new(1.0, -0.5, 0.0));

    let indices = vec!(0u, 1, 2,
                       0,  2, 3,
                       0,  3, 1);

    // Build the mesh.
    let mesh: Mesh3 = Mesh::new(Arc::new(points), Arc::new(indices), None, None);

    assert!(mesh.vertices().len() == 4);
}
