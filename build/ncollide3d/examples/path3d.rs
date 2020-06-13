/*

extern crate nalgebra as na;

use na::Point3;
use ncollide3d::procedural;
use ncollide3d::procedural::path::{ArrowheadCap, PolylinePath, PolylinePattern, StrokePattern};

fn main() {
    let control_points = [
        Point3::new(0.0f32, 1.0, 0.0),
        Point3::new(2.0, 4.0, 2.0),
        Point3::new(2.0, 1.0, 4.0),
        Point3::new(4.0, 4.0, 6.0),
        Point3::new(2.0, 1.0, 8.0),
        Point3::new(2.0, 4.0, 10.0),
        Point3::new(0.0, 1.0, 12.0),
        Point3::new(-2.0, 4.0, 10.0),
        Point3::new(-2.0, 1.0, 8.0),
        Point3::new(-4.0, 4.0, 6.0),
        Point3::new(-2.0, 1.0, 4.0),
        Point3::new(-2.0, 4.0, 2.0),
    ];

    // Setup the path.
    let bezier = procedural::bezier_curve(&control_points, 100);
    let mut path = PolylinePath::new(&bezier);

    // Setup the pattern.
    let start_cap = ArrowheadCap::new(1.5, 2.0, 0.0);
    let end_cap = ArrowheadCap::new(2.0, 2.0, 0.5);
    let pattern = ncollide2d::procedural::unit_circle(100);
    let mut pattern = PolylinePattern::new(pattern.coords(), true, start_cap, end_cap);

    // Stroke!
    let _ = pattern.stroke(&mut path);
}
*/
fn main() {}
