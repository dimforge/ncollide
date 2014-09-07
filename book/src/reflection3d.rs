extern crate "ncollide3df32" as ncollide;

use ncollide::implicit::HasMargin;
use ncollide::geom::{Cone, Reflection};

fn main() {
    let cone       = Cone::new(0.5, 0.75);
    let reflection = Reflection::new(&cone);

    // the reflection will inherit the cone default margin of 0.04
    assert!(reflection.margin() == 0.04);
}
