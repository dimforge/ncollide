extern crate "ncollide3df32" as ncollide;

use ncollide::geom::Cone;

fn main() {
    let cone = Cone::new(0.5, 0.75);

    assert!(cone.half_height() == 0.5);
    assert!(cone.radius() == 0.75);
}
