extern crate nalgebra as na;
extern crate ncollide2d;

use na::Point2;
use ncollide2d::shape::Polyline;

fn main() {
    let points = vec![
        Point2::new(0.0, 1.0),
        Point2::new(-1.0, -1.0),
        Point2::new(0.0, -0.5),
        Point2::new(1.0, -1.0),
        Point2::new(0.0, 1.0), // This forms a loop.
    ];

    // Build the polyline.
    let polyline = Polyline::new(points);

    assert!(polyline.vertices().len() == 5);
}
