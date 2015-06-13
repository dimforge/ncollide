//! Miscelaneous, unsorted generic geometric utilities.

#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(missing_docs)]
#![deny(unused_results)]
#![warn(unused_imports)]
#![allow(missing_copy_implementations)] // FIXME: deny this.
#![doc(html_root_url = "http://ncollide.org/doc")]

extern crate rand;
extern crate rustc_serialize;
extern crate num;
extern crate nalgebra as na;
extern crate ncollide_math as math;

pub use center::center;
pub use project_homogeneous::{project_homogeneous, project_homogeneous_to};
pub use triangle::{circumcircle, is_affinely_dependent_triangle3,
                   is_affinely_dependent_triangle, is_point_in_triangle, triangle_area,
                   triangle_center, triangle_perimeter};
pub use tetrahedron::{tetrahedron_volume, tetrahedron_signed_volume, tetrahedron_center};
pub use cleanup::remove_unused_points;
pub use derivatives::{dcos, dsin, binom};
pub use optimization::{maximize_with_newton, newton, minimize_with_bfgs, bfgs,
                       LineSearch, BacktrackingLineSearch};
pub use hashable_partial_eq::HashablePartialEq;
#[doc(inline)]
#[doc(inline)]
pub use as_bytes::AsBytes;
pub use cov::{cov, cov_and_center, center_reduce};
pub use median::median;
pub use sort::sort3;
pub use cross3::cross3;


pub mod data;
mod center;
mod project_homogeneous;
mod tetrahedron;
mod triangle;
mod cleanup;
mod derivatives;
mod optimization;
mod hashable_partial_eq;
#[doc(hidden)]
pub mod as_bytes;
mod cov;
mod median;
mod sort;
mod cross3;
