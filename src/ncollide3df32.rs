/*!
ncollide
========

**ncollide** is a n-dimensional collision detection library written with the
rust programming language.

As its name suggests, it is generic wrt the dimension: it works with both
2-dimensional and 3-dimensional geometries.  It might work with higher
dimensions (never tried).

## Compilation
You will need the last nightly build of the [rust compiler](http://www.rust-lang.org)
and the official package manager: [cargo](https://github.com/rust-lang/cargo).

Simply add the following to your `Cargo.toml` file:

```
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
#![deny(unnecessary_parens)]
#![deny(non_uppercase_statics)]
#![deny(unnecessary_qualification)]
#![deny(missing_doc)]
#![deny(unused_result)]
#![warn(unused_imports)]
#![deny(unnecessary_typecast)]
#![feature(macro_rules)]
#![feature(managed_boxes)]
#![feature(unsafe_destructor)]
#![feature(phase)]
#![allow(unused_imports)] // FIXME should be denied.
#![doc(html_root_url = "http://ncollide.org/doc")]

#[phase(plugin)] extern crate dim3;

extern crate std;
extern crate nalgebra;
extern crate sync;
extern crate serialize;
extern crate collections;
extern crate test;

// #[cfg(test)]
// extern crate rand;

pub mod bounding_volume;
pub mod geom;
pub mod ray;
pub mod narrow;
pub mod broad;
pub mod volumetric;
pub mod implicit;
pub mod parametric;
pub mod partitioning;
pub mod procedural;
pub mod utils;
pub mod data;

// #[cfg(test)]
// mod tests {
//     mod geom;
//     mod narrow;
//     mod algo;
// }

/// Compilation flags dependent aliases for mathematical types.
///
/// The aliases are selected, depending on the compilation flags. The possible flags are:
///
/// * `--cfg dim2` - use 2d vectors and matrices.
/// * `--cfg dim3` - use 3d vectors and matrices.
/// * `--cfg dim4` - use 4d vectors and matrices.
pub mod math {
    use nalgebra::na::{Vec3, Mat3, Rot3, Iso3};

    /// The scalar type.
    pub type Scalar = f32;

    /// The vector type.
    pub type Vect = Vec3<Scalar>;

    /// The orientation type.
    pub type Orientation = Vec3<Scalar>;

    /// The transformation matrix type.
    pub type Matrix = Iso3<Scalar>;

    /// The rotation matrix type.
    pub type RotationMatrix = Rot3<Scalar>;

    /// The inertia tensor type.
    pub type AngularInertia = Mat3<Scalar>;
}
