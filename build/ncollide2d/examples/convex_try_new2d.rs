extern crate nalgebra as na;

use na::Point2;
use ncollide2d::shape::ConvexPolygon;

fn main() {
    let points = vec![
        Point2::new(-1.0f32, 1.0),
        Point2::new(-0.5, -0.5),
        Point2::new(0.5, -0.5),
        Point2::new(1.0, 1.0),
    ];

    let convex = ConvexPolygon::try_new(points).expect("Invalid convex polygon.");
    assert!(convex.points().len() == 4);
}
