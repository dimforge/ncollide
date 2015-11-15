extern crate nalgebra as na;
extern crate ncollide;

use std::sync::Arc;
use na::Pnt2;
use ncollide::shape::Polyline;

fn main() {
    let points = vec!(
        Pnt2::new(0.0, 1.0),  Pnt2::new(-1.0, -1.0),
        Pnt2::new(0.0, -0.5), Pnt2::new(1.0, -1.0));

    let indices = vec!(Pnt2::new(0u32, 1),
                       Pnt2::new(1,  2),
                       Pnt2::new(2,  3),
                       Pnt2::new(3,  1));

    // Build the mesh.
    let mesh = Polyline::new(Arc::new(points), Arc::new(indices), None, None);

    assert!(mesh.vertices().len() == 4);
}
