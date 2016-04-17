# Getting started
**ncollide** uses the official Rust package manager [Cargo](http://crates.io)
for compilation and dependency resolution. Therefore, making **ncollide**
ready to be used by your project is simply a matter of adding a new dependency
to your `Cargo.toml` file.
```yml
[dependencies]
ncollide = "0.8.0"
```

Until **ncollide** reaches 1.0, it is strongly recommended to always use its
latest version, though you might encounter breaking changes from time to time.
Once your `Cargo.toml` file is set up, the corresponding crate must be imported
by your project with the regular `extern crate` directive:
```rust
extern crate ncollide;
```

## Cargo example
You may use this `Cargo.toml` file to compile the downloadable examples of this
guide. Simply replace `example.rs` by the actual example's file name.

#### Example <div class="btn-primary" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/cargo/Cargo.toml')"></div>
```
[package]
name    = "example-using-ncollide"
version = "0.0.0"
authors = [ "You" ]

[dependencies]
ncollide = "0.8.0"

[[bin]]
name = "example"
path = "./example.rs"
```

# Project structure
The **ncollide** crate is only an interface for several, smaller interdependent
crates part of the **ncollide** project. Thus if only a subset of the features
is of interest to you, you may depend on them directly:

Crate name                  | Description
----------------------------|-------------
**ncollide_math**           | Traits that must be satisfied by algebraic entities usable on the whole project. |
**ncollide_utils**          | [Miscellaneous](../miscellaneous) data structures and geometrical operations used by all the other crates. |
**ncollide_entities**       | [Shapes](../geometric_representations), [bounding volumes](../bounding_volumes), and spacial partitioning structures. |
**ncollide_queries**        | Complex geometric queries involving shapes defined by **ncollide_entities**. This includes contact point computation, proximity and point inclusion tests, ray-casting, etc. |
**ncollide_procedural**     | Procedural [mesh generation](mesh_generation/#mesh-generation) from parameters provided by the user. |
**ncollide_transformation** | Operators that creates an alternative geometrical representation of a shape given in input. This includes [convex hull](../mesh_generation/#primitives), [convex decomposition](../mesh_generation/#convex-decomposition), and smooth shapes discretization. |
**ncollide_pipeline**       | Persistent structures for recurrent (over time) geometric queries exploiting time-coherence. This exploits time-coherence and implements explicitly the concepts of [Broad Phase](collision_detection_pipeline/#broad-phase) and [Narrow Phase](collision_detection_pipeline/#narrow-phase). This also includes the [collision world](collision_detection_pipeline/#collision-world) which is the main interface between the user **ncollide**. |

To use any of those crates, simply add a corresponding dependency entry to your
`Cargo.toml`. Note that you should not expect the version numbers of those
crates to be identical. For example, **ncollide** being in version `0.8.0` does
not implies that **ncollide_queries** (say) is at its version `0.8.0` as well.
