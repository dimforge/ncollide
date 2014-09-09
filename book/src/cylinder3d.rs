extern crate "ncollide3df32" as ncollide;

use ncollide::geom::Cylinder;

fn main() {
    let cylinder = Cylinder::new(0.5, 1.0);

    assert!(cylinder.half_height() == 0.5);
    assert!(cylinder.radius() == 1.0);
}
