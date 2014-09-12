extern crate nalgebra;
extern crate "ncollide3df32" as ncollide;

use nalgebra::na::Vec3;
use ncollide::geom::Convex;

fn main() {
    let points = [
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 0.0)
    ];

    let convex = Convex::new(points);

    assert!(convex.pts().len() == 6);
}
