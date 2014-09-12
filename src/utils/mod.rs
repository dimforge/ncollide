//! Miscelaneous, unsorted generic geometric utilities.

pub use utils::center::center;
pub use utils::triangulate::triangulate;
pub use utils::project_homogeneous::{project_homogeneous, project_homogeneous_to};
pub use utils::triangle::{circumcircle, is_affinely_dependent_triangle, is_point_in_triangle,
                          triangle_area, triangle_perimeter};
pub use utils::tetrahedron::{tetrahedron_volume, tetrahedron_signed_volume, tetrahedron_center};
pub use utils::cleanup::remove_unused_points;
pub use utils::derivatives::{dcos, dsin, binom};
pub use utils::optimization::{maximize_with_newton, newton, minimize_with_bfgs, bfgs,
                              LineSearch, BacktrackingLineSearch};
pub use utils::hashable_partial_eq::HashablePartialEq;
#[doc(inline)]
pub use utils::any_private::AnyPrivate;
#[doc(inline)]
pub use utils::as_bytes::AsBytes;
pub use utils::cov::{cov, cov_and_center, center_reduce};
pub use utils::sort::sort3;


pub mod symbolic;
pub mod data;
mod center;
mod triangulate;
mod project_homogeneous;
mod tetrahedron;
mod triangle;
mod cleanup;
mod derivatives;
mod optimization;
mod hashable_partial_eq;
#[doc(hidden)]
pub mod any_private;
#[doc(hidden)]
pub mod as_bytes;
mod cov;
mod sort;
