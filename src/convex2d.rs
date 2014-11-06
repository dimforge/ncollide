extern crate "nalgebra" as na;
extern crate ncollide;

use na::Pnt2;
use ncollide::shape::Convex;

fn main() {
    let points = vec![
        Pnt2::new(-1.0f32, 1.0), Pnt2::new(-0.5, -0.5),
        Pnt2::new(0.0, 0.5),     Pnt2::new(0.5, -0.5),
        Pnt2::new(1.0, 1.0)
    ];

    let convex = Convex::new(points);

    // Convex does not compute explicitely the convex hull (which has 4 vertices)!
    assert!(convex.points().len() == 5);
}
