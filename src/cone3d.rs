extern crate ncollide;

use ncollide::shape::Cone;

fn main() {
    let cone = Cone::new(0.5f32, 0.75);

    assert!(cone.half_height() == 0.5);
    assert!(cone.radius() == 0.75);
}
