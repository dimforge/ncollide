extern crate nalgebra as na;
extern crate ncollide;

use na::Point3;
use ncollide::shape::ConvexHull;

fn main() {
    let points = vec![
        Point3::new(0.0f32, 0.0, 1.0),
        Point3::new(0.0, 0.0, -1.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, -1.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.0)
    ];

    let convex = ConvexHull::new(points);

    // ConvexHull does not compute explicitely the convex hull (which has 6 vertices)!
    assert!(convex.points().len() == 7);
}
