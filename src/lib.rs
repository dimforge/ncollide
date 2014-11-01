/*!
ncollide
========

**ncollide** is a 2 and 3-dimensional collision detection library written with the
rust programming language.

As its name suggests, it is generic wrt the dimension: it works with both
2-dimensional and 3-dimensional geometries.  It might work with higher
dimensions (never tried).

## Compilation
You will need the last nightly build of the [rust compiler](http://www.rust-lang.org)
and the official package manager: [cargo](https://github.com/rust-lang/cargo).

Simply add the following to your `Cargo.toml` file:

```.ignore
[dependencies.ncollide3df32]
git = "https://github.com/sebcrozet/ncollide"
```

## Features
- dynamic bounding volume tree based broad phase
- ball vs. ball collision detection,
- plane vs. any convex object collision detection.
- collision detection between arbitrary convex objects
- compound geometries
- ray-casting
- time of impact computation  for objects without rotational movement (compound vs. compound is not
  yet implemented)

And various traits for collision detectors and broad phase collision detection.

## What is missing
Some common features are still missing:

- heightmaps
*/

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![deny(unused_typecasts)]
#![feature(default_type_params)]
#![feature(macro_rules)]
#![feature(unsafe_destructor)]
#![feature(associated_types)]
#![feature(globs)]
#![doc(html_root_url = "http://ncollide.org/doc")]

extern crate "nalgebra" as na;
extern crate sync;
extern crate serialize;
extern crate collections;
extern crate test;

pub mod math;
pub mod bounding_volume;
pub mod shape;
pub mod ray;
pub mod narrow_phase;
pub mod broad;
pub mod support_map;
pub mod parametric;
pub mod partitioning;
pub mod procedural;
pub mod utils;
pub mod volumetric;

// #[cfg(test)]
// mod tests {
//     mod geom;
//     mod narrow;
//     mod algo;
// }
