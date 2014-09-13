# Getting started
**ncollide** uses the official Rust package manager [Cargo](http://crates.io)
for compilation and dependency resolution. Therefore, making **ncollide**
ready to be used by your project is simply a matter of adding a new dependency
to your `Cargo.toml` file (you do not even have to clone the repository
yourself!):
```rust
[dependencies.ncollide3df32]
git = "https://github.com/sebcrozet/ncollide"
```

Keep in mind that **ncollide** comes with multiple flavours. You might want to
adapt the previous dependency entry to select another version. For example, for
64-bit, 2D collision detection, use the following instead:
```rust
[dependencies.ncollide2df64]
git = "https://github.com/sebcrozet/ncollide"
```

Once your `Cargo.toml` file is set up, the corresponding crate must be imported
by your project using the regular `extern crate` directive:
```rust
extern crate ncollide3df32;
```

Again, if you want 64-bit, 2D collision detection, the adaptation is
straightforward:
```rust
extern crate ncollide2df64;
```


Having to write the dimension and the floating point precision used by the
library every time you import one of its functionality (e.g. `use
ncollide3df32::geom::Cuboid;`) might be very cumbersome. A nicer route is to
use an alias when the crate is imported:
```rust
extern crate "ncollide3df32" as ncollide;
```

This way, you only have to write statements like `use ncollide::geom::Cuboid;`.

## Cargo example
You may use one of those `Cargo.toml` files to compile the download examples of
this guide. Simply replace the words `example2d` and `example3d` by the real
name of the example.

###### 2D example <span class="d2" onclick="window.open('../src/cargo_2d/Cargo.toml')"></span>
```rust
[package]
name    = "example-using-ncollide2d"
version = "0.0.0"
authors = [ "You" ]

[dependencies.ncollide2df32]
git = "http://github.com/sebcrozet/ncollide"

[[bin]]
name = "example2d"
path = "./example2d.rs"
```

###### 3D example <span class="d3" onclick="window.open('../src/cargo_3d/Cargo.toml')"></span>
```rust
[package]
name    = "example-using-ncollide3d"
version = "0.0.0"
authors = [ "You" ]

[dependencies.ncollide3df32]
git = "http://github.com/sebcrozet/ncollide"

[[bin]]
name = "example3d"
path = "./example3d.rs"
```
