//! Collision shapes supported by ncollide.

pub use self::ball::Ball;
pub use self::plane::Plane;
pub use self::cuboid::Cuboid;
// pub use self::capsule::Capsule;
#[cfg(feature = "dim3")]
pub use self::cone::Cone;
#[cfg(feature = "dim3")]
pub use self::cylinder::Cylinder;
// pub use self::convex::ConvexHull;
// pub use self::convex_polygon::ConvexPolygon;
// pub use self::minkowski_sum::{cso_support_point, AnnotatedCSO, AnnotatedMinkowskiSum,
//                               AnnotatedPoint, MinkowskiSum, CSO};
// pub use self::reflection::Reflection;
// pub use self::compound::Compound;
// pub use self::base_mesh::{BaseMesh, BaseMeshElement};
// pub use self::trimesh::TriMesh;
// pub use self::polyline::Polyline;
pub use self::segment::{Segment, SegmentPointLocation};
// pub use self::triangle::{Triangle, TrianglePointLocation};
// pub use self::tetrahedron::{Tetrahedron, TetrahedronPointLocation};
// pub use self::torus::Torus;
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

// use na::{Isometry2, Isometry3, Point2, Point3, Vector2, Vector3};

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
// mod triangle;
// mod tetrahedron;
// mod base_mesh;
// mod trimesh;
// mod polyline;
mod ball;
// mod capsule;
#[cfg(feature = "dim3")]
mod cone;
#[cfg(feature = "dim3")]
mod cylinder;
// mod reflection;
// mod torus;
// mod compound;
// mod convex;
// mod convex_polygon;
#[cfg(feature = "dim2")]
mod convex_polyface2;
#[cfg(feature = "dim3")]
mod convex_polyface3;
// mod shape_impl;