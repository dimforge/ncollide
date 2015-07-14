# Getting started
**ncollide** uses the official Rust package manager [Cargo](http://crates.io)
for compilation and dependency resolution. Therefore, making **ncollide**
ready to be used by your project is simply a matter of adding a new dependency
to your `Cargo.toml` file (you do not even have to clone the repository
yourself!):
```rust
[dependencies]
ncollide = "*"
```

Once your `Cargo.toml` file is set up, the corresponding crate must be imported
by your project using the regular `extern crate` directive:
```rust
extern crate ncollide;
```

## Cargo example
You may use this `Cargo.toml` file to compile the downloadable examples of this
guide. Simply replace the words `example` by the real name of the example.

###### Example <span class="d" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/cargo/Cargo.toml')"></span>
```rust
[package]
name    = "example-using-ncollide"
version = "0.0.0"
authors = [ "You" ]

[dependencies]
ncollide = "*"

[[bin]]
name = "example"
path = "./example.rs"
```
