extern crate "nalgebra" as na;
extern crate "ncollide2df32" as ncollide;

use na::Pnt2;
use ncollide::geom::Convex;

fn main() {
    let points = [
        Pnt2::new(-1.0, 1.0), Pnt2::new(-0.5, -0.5),
        Pnt2::new(0.0, 0.5),  Pnt2::new(0.5, -0.5),
        Pnt2::new(1.0, 1.0)
    ];

    let convex = Convex::new(points.as_slice());

    assert!(convex.pts().len() == 4);
}
