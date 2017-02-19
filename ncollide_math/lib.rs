//! Trait implemented by the primitive algebraic types used by ncollide.

extern crate rustc_serialize;
extern crate num_traits as num;
extern crate rand;
extern crate approx;
extern crate alga;
extern crate nalgebra as na;

pub use point::Point;
pub use vector::Vector;
pub use isometry::Isometry;

mod point;
mod vector;
mod isometry;
