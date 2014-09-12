extern crate "ncollide2df32" as ncollide;

use ncollide::geom::{Cone, Reflection};

fn main() {
    let cone = Cone::new(0.5, 0.75);

    // Build the reflection.
    let _ = Reflection::new(&cone);
}
