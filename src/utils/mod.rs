//! Miscelaneous, unsorted generic geometric utilities.

pub use utils::center::center;
pub use utils::triangulate::triangulate;
pub use utils::project_homogeneous::{project_homogeneous, project_homogeneous_to};
pub use utils::triangle_utils::{circumcircle, is_affinely_dependent_triangle};


mod center;
mod triangulate;
mod project_homogeneous;
mod triangle_utils;
