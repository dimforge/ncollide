//! Persistent and time-coherent collision detection.

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![warn(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![deny(unused_typecasts)]
#![allow(missing_copy_implementations)]
#![feature(unsafe_destructor)]
#![feature(old_orphan_check)]
#![feature(unboxed_closures)]
#![doc(html_root_url = "http://ncollide.org/doc")]

extern crate test; // To compute the median.
extern crate "rustc-serialize" as rustc_serialize;
extern crate "nalgebra" as na;
extern crate "ncollide_math" as math;
extern crate "ncollide_utils" as utils;
extern crate "ncollide_entities" as entities;
extern crate "ncollide_queries" as queries;

pub mod broad_phase;
pub mod narrow_phase;
pub mod world;
