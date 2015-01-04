//! Geometric entities manipulated by ncollide.

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![warn(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![deny(unused_typecasts)]
#![feature(default_type_params)]
#![feature(macro_rules)]
#![feature(unsafe_destructor)]
#![feature(associated_types)]
#![feature(old_orphan_check)]
#![feature(globs)]
#![feature(phase)]
#![doc(html_root_url = "http://ncollide.org/doc")]

extern crate test; // To compute the median.
extern crate "rustc-serialize" as rustc_serialize;
extern crate "nalgebra" as na;
extern crate "ncollide_math" as math;
extern crate "ncollide_utils" as utils;
extern crate "ncollide_entities" as entities;
extern crate "ncollide_queries" as queries;
extern crate "ncollide_procedural" as procedural;


pub use to_trimesh::ToTriMesh;
pub use to_polyline::ToPolyline;
pub use hacd::hacd;
pub use convex_hull::{convex_hull3, convex_hull2, convex_hull2_idx};
pub use triangulate::triangulate;

mod to_trimesh;
mod to_polyline;
mod hacd;
mod convex_hull;
mod triangulate;
