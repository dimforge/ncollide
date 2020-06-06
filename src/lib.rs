/*!
ncollide
========

**ncollide** is a 2 and 3-dimensional collision detection library written with
the rust programming language.

As its name suggests, it is generic wrt the dimension: it works with both
2-dimensional and 3-dimensional shapes.

The official user guide is available [here](http://ncollide.org).
The rustdoc documentation is available [here](http://ncollide.org/rustdoc/ncollide).

## Compilation
You will need the last stable build of the [rust compiler](http://www.rust-lang.org)
and the official package manager: [cargo](https://github.com/rust-lang/cargo).

Simply add the following to your `Cargo.toml` file:

```.ignore
[dependencies]
ncollide2d = "0.23" # For 2D collision detection.
ncollide3d = "0.23" # For 3D collision detection.
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
#![deny(missing_docs)] // FIXME: should be denied.
#![deny(unused_results)]
#![warn(unused_imports)]
#![allow(missing_copy_implementations)]
#![doc(html_root_url = "http://ncollide.org/rustdoc")]

#[cfg(feature = "serde")]
#[macro_use]
extern crate serde;
#[macro_use]
extern crate approx;
#[macro_use]
extern crate downcast_rs;
#[macro_use]
extern crate bitflags;
extern crate nalgebra as na;
extern crate num_traits as num;

pub use nalgebra;
pub use simba;

macro_rules! try_ret {
    ($val: expr) => {
        try_ret!($val, ())
    };
    ($val: expr, $ret: expr) => {
        if let Some(val) = $val {
            val
        } else {
            return $ret;
        }
    };
}

const NOT_REGISTERED_ERROR: &'static str =
    "This collision object has not been registered into a world (proxy indexes are None).";

#[deprecated = "use the `pipeline` module instead."]
pub use crate::pipeline::{broad_phase, narrow_phase, world};

pub mod bounding_volume;
pub mod interpolation;
pub mod partitioning;
pub mod pipeline;
pub mod procedural;
pub mod query;
pub mod shape;
pub mod transformation;
pub mod utils;

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "dim3")]
pub mod math {
    use na::{Isometry3, Matrix3, Point3, Translation3, UnitQuaternion, Vector3, Vector6, U3, U6};

    /// The dimension of the space.
    pub const DIM: usize = 3;

    /// The dimension of the ambient space.
    pub type Dim = U3;

    /// The dimension of a spatial vector.
    pub type SpatialDim = U6;

    /// The dimension of the rotations.
    pub type AngularDim = U3;

    /// The point type.
    pub type Point<N> = Point3<N>;

    /// The angular vector type.
    pub type AngularVector<N> = Vector3<N>;

    /// The vector type.
    pub type Vector<N> = Vector3<N>;

    /// The matrix type.
    pub type Matrix<N> = Matrix3<N>;

    /// The vector type with dimension `SpatialDim × 1`.
    pub type SpatialVector<N> = Vector6<N>;

    /// The orientation type.
    pub type Orientation<N> = Vector3<N>;

    /// The transformation matrix type.
    pub type Isometry<N> = Isometry3<N>;

    /// The rotation matrix type.
    pub type Rotation<N> = UnitQuaternion<N>;

    /// The translation type.
    pub type Translation<N> = Translation3<N>;
}

/// Compilation flags dependent aliases for mathematical types.
#[cfg(feature = "dim2")]
pub mod math {
    use na::{Isometry2, Matrix2, Point2, Translation2, UnitComplex, Vector1, Vector2, U2};

    /// The dimension of the space.
    pub const DIM: usize = 2;

    /// The dimension of the ambiant space.
    pub type Dim = U2;

    /// The point type.
    pub type Point<N> = Point2<N>;

    /// The vector type.
    pub type Vector<N> = Vector2<N>;

    /// The matrix type.
    pub type Matrix<N> = Matrix2<N>;

    /// The orientation type.
    pub type Orientation<N> = Vector1<N>;

    /// The transformation matrix type.
    pub type Isometry<N> = Isometry2<N>;

    /// The rotation matrix type.
    pub type Rotation<N> = UnitComplex<N>;

    /// The translation type.
    pub type Translation<N> = Translation2<N>;
}
