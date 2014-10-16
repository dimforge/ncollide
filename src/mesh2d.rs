extern crate "nalgebra" as na;
extern crate "ncollide2df32" as ncollide;

use std::sync::Arc;
use na::Pnt2;
use ncollide::geom::Mesh;

fn main() {
    let points = vec!(
        Pnt2::new(0.0, 1.0),  Pnt2::new(-1.0, -1.0),
        Pnt2::new(0.0, -0.5), Pnt2::new(1.0, -1.0));

    let indices = vec!(0u, 1,
                       1,  2,
                       2,  3,
                       3,  1);

    // Build the mesh.
    let mesh = Mesh::new(Arc::new(points), Arc::new(indices), None, None);

    assert!(mesh.vertices().len() == 4);
}
