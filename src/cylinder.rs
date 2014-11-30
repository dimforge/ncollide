extern crate ncollide;

use ncollide::shape::Cylinder;

fn main() {
    let cylinder = Cylinder::new(0.5f32, 1.0);

    assert!(cylinder.half_height() == 0.5);
    assert!(cylinder.radius() == 1.0);
}
