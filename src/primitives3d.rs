extern crate "nalgebra" as na;
extern crate ncollide;

use std::rand;
use na::Pnt3;
use ncollide::procedural;

fn main() {
    let mut points = Vec::new();
    for _ in range(0u, 100000) {
        points.push(rand::random::<Pnt3<f32>>() * 2.0f32);
    }

    let _ = procedural::convex_hull3(points.as_slice());
}
