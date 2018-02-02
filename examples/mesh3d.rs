extern crate nalgebra as na;
extern crate ncollide;

use std::sync::Arc;
use na::Point3;
use ncollide::shape::TriMesh;

fn main() {
    let points = vec![
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(-1.0, -0.5, 0.0),
        Point3::new(0.0, -0.5, -1.0),
        Point3::new(1.0, -0.5, 0.0),
    ];

    let indices = vec![
        Point3::new(0usize, 1, 2),
        Point3::new(0, 2, 3),
        Point3::new(0, 3, 1),
    ];

    // Build the mesh.
    let mesh = TriMesh::new(Arc::new(points), Arc::new(indices), None, None);

    assert!(mesh.vertices().len() == 4);
}
