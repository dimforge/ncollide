extern crate ncollide;

use ncollide::shape::{Cone, Reflection};

fn main() {
    let cone = Cone::new(0.5f32, 0.75);

    // Build the reflection.
    let _ = Reflection::new(&cone);
}
