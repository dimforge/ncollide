//! Transformation, simplification and decomposition of meshes.

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
extern crate ncollide_geometry as geometry;
extern crate ncollide_procedural as procedural;


pub use to_trimesh::ToTriMesh;
pub use to_polyline::ToPolyline;
pub use hacd::hacd;
pub use convex_hull3::convex_hull3;
pub use convex_hull2::{convex_hull2, convex_hull2_idx};
pub use triangulate::triangulate;

mod to_trimesh;
mod to_polyline;
mod hacd;
#[doc(hidden)]
pub mod convex_hull_utils; // Internal implementation details.
mod convex_hull2;
mod convex_hull3;
mod triangulate;
