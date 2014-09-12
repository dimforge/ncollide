extern crate nalgebra;
extern crate "ncollide2df32" as ncollide;

use nalgebra::na::Vec2;
use ncollide::geom::Convex;

fn main() {
    let points = [
        Vec2::new(-1.0, 1.0), Vec2::new(-0.5, -0.5),
        Vec2::new(0.0, 0.5),  Vec2::new(0.5, -0.5),
        Vec2::new(1.0, 1.0)
    ];

    let convex = Convex::new(points.as_slice());

    assert!(convex.pts().len() == 4);
}
