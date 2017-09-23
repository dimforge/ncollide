//! Persistent and time-coherent collision detection.

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![warn(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![allow(missing_copy_implementations)]
#![doc(html_root_url = "http://ncollide.org/rustdoc")]

extern crate rustc_serialize;
extern crate alga;
extern crate nalgebra as na;
extern crate ncollide_math as math;
extern crate ncollide_utils as utils;
extern crate ncollide_geometry as geometry;

pub mod broad_phase;
pub mod narrow_phase;
pub mod world;
pub mod events;
