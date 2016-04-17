extern crate nalgebra as na;
extern crate ncollide;

use std::sync::Arc;
use na::Point2;
use ncollide::shape::Polyline;

fn main() {
    let points = vec!(
        Point2::new(0.0, 1.0),  Point2::new(-1.0, -1.0),
        Point2::new(0.0, -0.5), Point2::new(1.0, -1.0));

    let indices = vec!(Point2::new(0usize, 1),
                       Point2::new(1,  2),
                       Point2::new(2,  3),
                       Point2::new(3,  1));

    // Build the mesh.
    let mesh = Polyline::new(Arc::new(points), Arc::new(indices), None, None);

    assert!(mesh.vertices().len() == 4);
}
