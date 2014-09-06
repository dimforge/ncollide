# Getting started
**ncollide** uses the official Rust package manager [cargo](http://crates.io)
for compilation and dependency resolution. Therefore, making **ncollide**
ready to be used by your project is simply a matter of adding a new dependency
to your `Cargo.toml` file (you do not even have to clone the repository
yourself!):
```rust
[dependencies.ncollide3df32]
git = "https://github.com/sebcrozet/ncollide"
```

Keep in mind that **ncollide** comes in multiple flavours. You might want to
adapt the previous dependency entry to selected another version. For example,
for 32-bit, 2D collision detection, use the following instead:
```rust
[dependencies.ncollide2df32]
git = "https://github.com/sebcrozet/ncollide"
```

Once your `Cargo.toml` file is set up, the corresponding crate must be imported
by your project using the regular `extern crate` directive:
```rust
extern crate ncollide3df32;
```

Again, if you want 32-bit, 2D collision detection, the adaptation is
straightforward:
```rust
extern crate ncollide2df32;
```


Having to write the dimension and the floating point precision used by the
library every time you import one of its functionality (e.g. `use
ncollide3df32::geom::Cuboid;`) might be very cumbersome. A nicer route is to
use an alias when the crate is imported:
```rust
extern crate ncollide = "ncollide3df32";
```

This way, you only have to write statements like `use ncollide::geom::Cuboid;`.

###### Working example <button style="float:right;" class="btn btn-primary" type="button" id="download-code" onclick="window.open('../src/getting_started.rs')"><img style="float:left;width:20px;height:20px;" src="../img/d.svg" /></button>
```rust
extern crate nalgebra;
extern crate ncollide = "ncollide3df32";

use nalgebra::na::Vec3;
use ncollide::geom::Cuboid;
use ncollide::ray::{Ray, RayCast};

fn main() {
    let cube = Cuboid::new(Vec3::new(1.0f32, 1.0, 1.0));
    let ray  = Ray::new(Vec3::new(0.0f32, 0.0, -1.0), Vec3::z());

    assert!(cube.intersects_ray(&ray));
}
```
