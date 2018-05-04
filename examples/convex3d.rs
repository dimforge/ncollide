extern crate nalgebra as na;
extern crate ncollide3d;

use na::Point3;
use ncollide3d::shape::ConvexHull;

fn main() {
    let points = vec![
        Point3::new(0.0f32, 0.0, 1.0),
        Point3::new(0.0, 0.0, -1.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, -1.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
    ];

    let convex = ConvexHull::new(points);
    convex.check_geometry();
}
