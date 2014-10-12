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
#![deny(unnecessary_parens)]
#![deny(non_uppercase_statics)]
#![deny(unnecessary_qualification)]
#![deny(missing_doc)]
#![deny(unused_result)]
#![warn(unused_imports)]
#![deny(unnecessary_typecast)]
#![feature(default_type_params)]
#![feature(macro_rules)]
#![feature(unsafe_destructor)]
#![doc(html_root_url = "http://ncollide.org/doc")]

extern crate "nalgebra" as na;
extern crate sync;
extern crate serialize;
extern crate collections;
extern crate test;
extern crate debug;

pub mod bounding_volume;
pub mod geom;
pub mod ray;
pub mod narrow;
pub mod broad;
pub mod implicit;
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

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "3d")]
pub mod math {
    use na::{Pnt3, Vec3, Mat3, Rot3, Iso3};

    /// The scalar type.
    #[cfg(feature = "f32")]
    pub type Scalar = f32;

    /// The scalar type.
    #[cfg(feature = "f64")]
    pub type Scalar = f64;

    /// The point type.
    pub type Point = Pnt3<Scalar>;

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

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "2d")]
pub mod math {
    use na::{Pnt2, Vec1, Vec2, Mat1, Rot2, Iso2};

    /// The scalar type.
    #[cfg(feature = "f32")]
    pub type Scalar = f32;

    /// The scalar type.
    #[cfg(feature = "f64")]
    pub type Scalar = f64;

    /// The point type.
    pub type Point = Pnt2<Scalar>;

    /// The vector type.
    pub type Vect = Vec2<Scalar>;

    /// The orientation type.
    pub type Orientation = Vec1<Scalar>;

    /// The transformation matrix type.
    pub type Matrix = Iso2<Scalar>;

    /// The rotation matrix type.
    pub type RotationMatrix = Rot2<Scalar>;

    /// The inertia tensor type.
    pub type AngularInertia = Mat1<Scalar>;
}

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "4d")]
pub mod math {
    use na::{Pnt4, Vec4, Mat4, Rot4, Iso4};

    /// The scalar type.
    #[cfg(feature = "f32")]
    pub type Scalar = f32;

    /// The scalar type.
    #[cfg(feature = "f64")]
    pub type Scalar = f64;

    /// The point type.
    pub type Point = Pnt4<Scalar>;

    /// The vector type.
    pub type Vect = Vec4<Scalar>;

    /// The orientation type.
    pub type Orientation = Vec4<Scalar>;

    /// The transformation matrix type.
    pub type Matrix = Iso4<Scalar>;

    /// The rotation matrix type.
    pub type RotationMatrix = Rot4<Scalar>;

    /// The inertia tensor type.
    pub type AngularInertia = Mat4<Scalar>;
}
