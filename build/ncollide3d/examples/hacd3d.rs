/*
extern crate nalgebra as na;
extern crate ncollide2d;
extern crate ncollide3d; // Needed here only for path generation.

use na::Point3;
use ncollide3d::procedural;
use ncollide3d::procedural::path::{NoCap, PolylinePath, PolylinePattern, StrokePattern};
use ncollide3d::transformation;

fn main() {
    /*
     * Path stroke.
     */
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
    let bezier = procedural::bezier_curve(&control_points, 100);
    let mut path = PolylinePath::new(&bezier);
    let pattern = ncollide2d::procedural::unit_circle(100);
    let mut pattern = PolylinePattern::new(pattern.coords(), true, NoCap::new(), NoCap::new());
    let mut trimesh = pattern.stroke(&mut path);

    // The path stroke does not generate normals =(
    // Compute them as they are needed by the HACD.
    trimesh.recompute_normals();

    /*
     * Decomposition of the mesh.
     */
    let (decomp, partitioning) = transformation::hacd(trimesh.clone(), 0.03, 0);

    // We end up with 7 convex parts.
    assert!(decomp.len() == 7);
    assert!(partitioning.len() == 7);
}
*/

fn main() {}
