extern crate nalgebra as na;

use na::Point2;
use ncollide2d::transformation;

fn main() {
    let mut points = Vec::new();
    for _ in 0usize..100000 {
        points.push(rand::random::<Point2<f32>>() * 2.0);
    }

    let _ = transformation::convex_hull(&points[..]);
}
