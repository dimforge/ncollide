extern crate ncollide = "ncollide3df32";

use ncollide::geom::Capsule;

fn main() {
    let capsule = Capsule::new(0.5, 0.75);

    assert!(capsule.half_height() == 0.5);
    assert!(capsule.radius() == 0.75);
}
