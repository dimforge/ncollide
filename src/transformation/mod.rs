//! Transformation, simplification and decomposition of meshes.

#[cfg(feature = "dim3")]
pub use self::to_trimesh::ToTriMesh;
#[cfg(feature = "dim2")]
pub use self::to_polyline::ToPolyline;
pub use self::hacd::hacd;
#[cfg(feature = "dim3")]
pub use self::convex_hull3::convex_hull3 as convex_hull;
#[cfg(feature = "dim3")]
pub(crate) use self::convex_hull2::convex_hull2_idx;
#[cfg(feature = "dim2")]
pub use self::convex_hull2::{convex_hull2 as conve_hull, convex_hull2_idx as convex_hull_idx};
// pub use self::triangulate::triangulate;

#[cfg(feature = "dim3")]
mod to_trimesh;
#[cfg(feature = "dim2")]
mod to_polyline;
#[cfg(feature = "dim3")]
mod hacd;
#[doc(hidden)]
pub mod convex_hull_utils; // Internal implementation details.
mod convex_hull2;
#[cfg(feature = "dim3")]
mod convex_hull3;
// mod triangulate;
