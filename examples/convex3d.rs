extern crate nalgebra as na;
extern crate ncollide;

use na::Pnt3;
use ncollide::shape::Convex;

fn main() {
    let points = vec![
        Pnt3::new(0.0f32, 0.0, 1.0),
        Pnt3::new(0.0, 0.0, -1.0),
        Pnt3::new(0.0, 1.0, 0.0),
        Pnt3::new(0.0, -1.0, 0.0),
        Pnt3::new(1.0, 0.0, 0.0),
        Pnt3::new(-1.0, 0.0, 0.0),
        Pnt3::new(0.0, 0.0, 0.0)
    ];

    let convex = Convex::new(points);

    // Convex does not compute explicitely the convex hull (which has 6 vertices)!
    assert!(convex.points().len() == 7);
}
