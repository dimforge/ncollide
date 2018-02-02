//! Trait implemented by the primitive algebraic types used by ncollide.

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![doc(html_root_url = "http://ncollide.org/rustdoc")]

extern crate alga;
extern crate approx;
extern crate nalgebra as na;
extern crate num_traits as num;
extern crate rand;
extern crate rustc_serialize;

pub use point::Point;
pub use vector::Vector;
pub use isometry::Isometry;

mod point;
mod vector;
mod isometry;
