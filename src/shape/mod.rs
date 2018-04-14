//! Collision shapes supported by ncollide.

pub use self::ball::Ball;
pub use self::plane::Plane;
pub use self::cuboid::Cuboid;
pub use self::capsule::Capsule;
#[cfg(feature = "dim3")]
pub use self::cone::Cone;
#[cfg(feature = "dim3")]
pub use self::cylinder::Cylinder;
#[cfg(feature = "dim3")]
pub use self::convex::ConvexHull;
#[cfg(feature = "dim2")]
pub use self::convex_polygon::ConvexPolygon;
// pub use self::minkowski_sum::{cso_support_point, AnnotatedCSO, AnnotatedMinkowskiSum,
//                               AnnotatedPoint, MinkowskiSum, CSO};
// pub use self::reflection::Reflection;
pub use self::compound::Compound;
// pub use self::base_mesh::{BaseMesh, BaseMeshElement};
// pub use self::trimesh::TriMesh;
// pub use self::polyline::Polyline;
pub use self::segment::{Segment, SegmentPointLocation};
#[cfg(feature = "dim3")]
pub use self::triangle::{Triangle, TrianglePointLocation};
#[cfg(feature = "dim3")]
pub use self::tetrahedron::{Tetrahedron, TetrahedronPointLocation};
#[cfg(feature = "dim2")]
pub use self::convex_polyface2::ConvexPolyface;
#[cfg(feature = "dim3")]
pub use self::convex_polyface3::ConvexPolyface;
#[doc(inline)]
pub use self::composite_shape::CompositeShape;
#[doc(inline)]
pub use self::support_map::SupportMap;
pub use self::convex_polyhedron::{ConvexPolyhedron, FeatureId};
#[doc(inline)]
pub use self::shape::{Shape, ShapeHandle};

#[doc(hidden)]
pub mod composite_shape;
#[doc(hidden)]
pub mod support_map;
mod convex_polyhedron;
#[doc(hidden)]
pub mod shape;
mod plane;
mod cuboid;
// mod minkowski_sum;
mod segment;
#[cfg(feature = "dim3")]
mod triangle;
#[cfg(feature = "dim3")]
mod tetrahedron;
// mod base_mesh;
// mod trimesh;
// mod polyline;
mod ball;
mod capsule;
#[cfg(feature = "dim3")]
mod cone;
#[cfg(feature = "dim3")]
mod cylinder;
// mod reflection;
// mod torus;
mod compound;
#[cfg(feature = "dim3")]
mod convex;
#[cfg(feature = "dim2")]
mod convex_polygon;
#[cfg(feature = "dim2")]
mod convex_polyface2;
#[cfg(feature = "dim3")]
mod convex_polyface3;
// mod shape_impl;