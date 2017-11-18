/*!
ncollide
========

**ncollide** is a 2 and 3-dimensional collision detection library written with
the rust programming language.

As its name suggests, it is generic wrt the dimension: it works with both
2-dimensional and 3-dimensional shapes. It might work with higher
dimensions (never tried).

The official user guide is available [here](http://ncollide.org).
The rustdoc documentation is available [here](http://ncollide.org/rustdoc/ncollide).

## Compilation
You will need the last stable build of the [rust compiler](http://www.rust-lang.org)
and the official package manager: [cargo](https://github.com/rust-lang/cargo).

Simply add the following to your `Cargo.toml` file:

```.ignore
[dependencies]
ncollide = "0.13"
```


## Features
- dynamic bounding volume tree based broad phase
- ball vs. ball collision detection,
- plane vs. any convex object collision detection.
- collision detection between arbitrary convex objects
- compound shapes
- ray-casting
- time of impact computation  for objects without rotational movement (compound vs. compound is not
  yet implemented)

And various traits for collision detectors and broad phase collision detection.
*/

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![warn(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![allow(missing_copy_implementations)]
#![doc(html_root_url = "http://ncollide.org/rustdoc")]

pub extern crate ncollide_math;
pub extern crate ncollide_utils;
pub extern crate ncollide_geometry;
pub extern crate ncollide_pipeline;
pub extern crate ncollide_procedural;
pub extern crate ncollide_transformation;

pub use ncollide_math as math;
pub use ncollide_utils as utils;
pub use ncollide_geometry::{shape, bounding_volume, partitioning, query};
pub use ncollide_pipeline::{narrow_phase, broad_phase, world, events};
pub use ncollide_procedural as procedural;
pub use ncollide_transformation as transformation;
