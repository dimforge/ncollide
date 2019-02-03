//! Transformation, simplification and decomposition of meshes.

#[cfg(feature = "dim3")]
pub(crate) use self::convex_hull2::convex_hull2_idx;
#[cfg(feature = "dim2")]
pub use self::convex_hull2::{convex_hull2 as convex_hull, convex_hull2_idx as convex_hull_idx};
#[cfg(feature = "dim3")]
pub use self::convex_hull3::convex_hull3 as convex_hull;
#[cfg(feature = "dim3")]
pub use self::hacd::hacd;
#[cfg(feature = "dim2")]
pub use self::to_polyline::ToPolyline;
#[cfg(feature = "dim3")]
pub use self::to_trimesh::ToTriMesh;
// pub use self::triangulate::triangulate;

mod convex_hull2;
#[cfg(feature = "dim3")]
mod convex_hull3;
#[doc(hidden)]
pub mod convex_hull_utils; // Internal implementation details.
#[cfg(feature = "dim3")]
mod hacd;
#[cfg(feature = "dim2")]
mod to_polyline;
#[cfg(feature = "dim3")]
mod to_trimesh;
// mod triangulate;
