/*!
ncollide  [![Build Status](https://travis-ci.org/sebcrozet/ncollide.png?branch=master)](https://travis-ci.org/sebcrozet/ncollide)
========

**ncollide** is a n-dimensional collision detection library written with the
rust programming language.

As its name suggests, it is generic wrt the dimension: it works with both
2-dimensional and 3-dimensional geometries.  It might work with higher
dimensions (never tried).

An on-line version of this documentation is available [here](http://www.rust-ci.org/sebcrozet/ncollide/doc/ncollide2df32/index.html).

## Compilation
You will need the last rust compiler from the master branch.
If you encounter problems, make sure you have the last version before creating an issue.

The simplest way to build **ncollide** and all its dependencies is to do a
recursive clone:


    git clone --recursive git://github.com/sebcrozet/ncollide.git
    cd ncollide
    make deps
    make

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

#[crate_id = "ncollide2df32#0.1"];
#[crate_type = "lib"];
#[deny(non_camel_case_types)];
#[deny(unnecessary_parens)];
#[deny(non_uppercase_statics)];
#[deny(unnecessary_qualification)];
#[deny(missing_doc)];
#[deny(unused_result)];
#[feature(macro_rules)];
#[feature(managed_boxes)];
#[doc(html_root_url = "http://www.rust-ci.org/sebcrozet/ncollide/doc")];

extern mod std;
extern mod extra;
extern mod nalgebra;
extern mod sync;
extern mod serialize;

pub mod bounding_volume;
pub mod geom;
pub mod ray;
pub mod narrow;
pub mod broad;
pub mod contact;
pub mod volumetric;
pub mod implicit;
pub mod partitioning;

/// Data structure utilities.
pub mod util;

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
/// * `--cfg f32`  - use 32-bit floating point values.
/// * `--cfg f64`  - use 64-bit floating point values.
pub mod math {
    use nalgebra::na::{Vec1, Vec2, Mat1, Iso2};

    /// The scalar type.
    pub type Scalar = f32;

    /// The scalar type.
    pub type N      = Scalar;

    /// The vector type.
    pub type V      = Vec2<Scalar>;

    /// The linear velocity or position (center of mass) type.
    pub type LV     = Vec2<Scalar>;

    /// The angular velocity or orientation (center of mass) type.
    pub type AV     = Vec1<Scalar>;

    /// The transformation matrix type.
    pub type M      = Iso2<Scalar>;

    /// The inertia tensor type.
    pub type II     = Mat1<Scalar>;
}
