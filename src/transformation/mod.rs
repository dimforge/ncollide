//! Transformation, simplification and decomposition of meshes.

#[cfg(feature = "dim3")]
pub use self::trimesh::TriMesh;
// pub use self::to_trimesh::ToTriMesh;
// pub use self::to_polyline::ToPolyline;
// pub use self::hacd::hacd;
#[cfg(feature = "dim3")]
pub use self::convex_hull3::convex_hull3;
#[cfg(feature = "dim2")]
pub use self::convex_hull2::{convex_hull2, convex_hull2_idx};
// pub use self::triangulate::triangulate;

#[cfg(feature = "dim3")]
mod trimesh;
// mod to_trimesh;
// mod to_polyline;
// mod hacd;
#[doc(hidden)]
pub mod convex_hull_utils; // Internal implementation details.
#[cfg(feature = "dim2")]
mod convex_hull2;
#[cfg(feature = "dim3")]
mod convex_hull3;
// mod triangulate;
