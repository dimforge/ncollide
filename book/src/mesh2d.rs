extern crate nalgebra;
extern crate "ncollide2df32" as ncollide;

use std::sync::Arc;
use nalgebra::na::Vec2;
use ncollide::geom::Mesh;

fn main() {
    let points = vec!(
        Vec2::new(0.0, 1.0),  Vec2::new(-1.0, -0.5),
        Vec2::new(0.0, -0.5), Vec2::new(1.0, -0.5)
        );

    let indices = vec!(0u, 1,
                       1,  2,
                       2,  3,
                       3,  1);

    // Build the mesh.
    let mesh = Mesh::new(Arc::new(points), Arc::new(indices), None, None);

    assert!(mesh.margin() == 0.04); // Meshes have a margin too!
}
