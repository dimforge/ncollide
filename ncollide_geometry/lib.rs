//! Geometric entities manipulated by ncollide and operations on them.

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![warn(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![doc(html_root_url = "http://ncollide.org/rustdoc")]

extern crate rustc_serialize;
extern crate num_traits as num;
#[macro_use]
extern crate approx;
extern crate alga;
extern crate nalgebra as na;
extern crate ncollide_math as math;
extern crate ncollide_utils as utils;

pub mod shape;
pub mod bounding_volume;
pub mod partitioning;
pub mod query;
