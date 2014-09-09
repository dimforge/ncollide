extern crate nalgebra;
extern crate "ncollide3df32" as ncollide;

use std::sync::Arc;
use nalgebra::na::Vec3;
use ncollide::geom::Mesh;

fn main() {
    let points = vec!(
        Vec3::new(0.0, 1.0, 0.0),   Vec3::new(-1.0, -0.5, 0.0),
        Vec3::new(0.0, -0.5, -1.0), Vec3::new(1.0, -0.5, 0.0)
        );

    let indices = vec!(0u, 1, 2,
                       0,  2, 3,
                       0,  3, 1);

    // Build the mesh.
    let mesh = Mesh::new(Arc::new(points), Arc::new(indices), None, None);

    assert!(mesh.vertices().len() == 4);
}
