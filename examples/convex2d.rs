extern crate nalgebra as na;
extern crate ncollide;

use na::Point2;
use ncollide::shape::ConvexHull;

fn main() {
    let points = vec![
        Point2::new(-1.0f32, 1.0),
        Point2::new(-0.5, -0.5),
        Point2::new(0.0, 0.5),
        Point2::new(0.5, -0.5),
        Point2::new(1.0, 1.0),
    ];

    let convex = ConvexHull::new(points);

    // ConvexHull does not compute explicitely the convex hull (which has 4 vertices)!
    assert!(convex.points().len() == 5);
}
