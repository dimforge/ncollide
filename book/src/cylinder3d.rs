extern crate "ncollide3df32" as ncollide;

use ncollide::geom::Cylinder;

fn main() {
    let cylinder1 = Cylinder::new(0.5, 1.0);
    let cylinder2 = Cylinder::new_with_margin(0.5, 1.0, 0.2);

    assert!(cylinder1.margin() == 0.04); // default margin
    assert!(cylinder2.margin() == 0.2);  // user-defined margin
}
