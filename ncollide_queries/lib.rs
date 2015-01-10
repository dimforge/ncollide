//! Geometric queries.

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![warn(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![deny(unused_typecasts)]
#![feature(unsafe_destructor)]
#![feature(plugin)]
#![feature(old_impl_check)]
#![feature(old_orphan_check)]
#![doc(html_root_url = "http://ncollide.org/doc")]


extern crate test; // To compute the median.
extern crate "rustc-serialize" as rustc_serialize;
extern crate "nalgebra" as na;
extern crate "ncollide_math" as math;
extern crate "ncollide_utils" as utils;
#[plugin]
extern crate ncollide_entities;
extern crate "ncollide_entities" as entities;
extern crate "ncollide_procedural" as procedural;

pub mod geometry;
pub mod point;
pub mod ray;
