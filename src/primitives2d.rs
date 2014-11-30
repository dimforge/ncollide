extern crate "nalgebra" as na;
extern crate ncollide;

use std::rand;
use na::Pnt2;
use ncollide::procedural;

fn main() {
    let mut points = Vec::new();
    for _ in range(0u, 100000) {
        points.push(rand::random::<Pnt2<f32>>() * 2.0);
    }

    let _ = procedural::convex_hull2(points.as_slice());
}
