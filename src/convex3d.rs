extern crate "nalgebra" as na;
extern crate "ncollide3df32" as ncollide;

use na::Pnt3;
use ncollide::geom::Convex;

fn main() {
    let points = [
        Pnt3::new(0.0, 0.0, 1.0),
        Pnt3::new(0.0, 0.0, -1.0),
        Pnt3::new(0.0, 1.0, 0.0),
        Pnt3::new(0.0, -1.0, 0.0),
        Pnt3::new(1.0, 0.0, 0.0),
        Pnt3::new(-1.0, 0.0, 0.0),
        Pnt3::new(0.0, 0.0, 0.0)
    ];

    let convex = Convex::new(points);

    assert!(convex.pts().len() == 6);
}
