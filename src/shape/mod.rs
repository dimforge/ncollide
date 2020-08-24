//! Collision shapes supported by ncollide.

pub use self::ball::Ball;
pub use self::capsule::Capsule;
#[doc(inline)]
pub use self::composite_shape::CompositeShape;
pub use self::compound::Compound;
#[cfg(feature = "dim3")]
pub use self::cone::Cone;
#[cfg(feature = "dim3")]
pub use self::convex::ConvexHull;
#[cfg(feature = "dim2")]
pub use self::convex_polygon::ConvexPolygon;
#[cfg(feature = "dim2")]
pub use self::convex_polygonal_feature2::ConvexPolygonalFeature;
#[cfg(feature = "dim3")]
pub use self::convex_polygonal_feature3::{ClippingCache, ConvexPolygonalFeature};
pub use self::convex_polyhedron::{ConvexPolyhedron, FeatureId};
pub use self::cuboid::Cuboid;
#[cfg(feature = "dim3")]
pub use self::cylinder::Cylinder;
pub use self::deformable_shape::{DeformableShape, DeformationsType};
//#[cfg(feature = "dim3")]
//pub use self::deformable_trimesh::DeformableTriMesh;
#[cfg(feature = "dim2")]
pub use self::heightfield2::HeightField;
#[cfg(feature = "dim3")]
pub use self::heightfield3::{HeightField, HeightFieldCellStatus};
pub use self::plane::Plane;
pub use self::polyline::Polyline;
pub use self::segment::{Segment, SegmentPointLocation};
#[doc(inline)]
pub use self::shape::{Shape, ShapeHandle};
#[doc(inline)]
pub use self::support_map::SupportMap;
#[cfg(feature = "dim3")]
pub use self::tetrahedron::{Tetrahedron, TetrahedronPointLocation};
pub use self::triangle::{Triangle, TrianglePointLocation};
#[cfg(feature = "dim3")]
pub use self::trimesh::{FaceAdjacentToEdge, TriMesh, TriMeshEdge, TriMeshFace, TriMeshVertex};

mod ball;
mod capsule;
#[doc(hidden)]
pub mod composite_shape;
mod compound;
#[cfg(feature = "dim3")]
mod cone;
#[cfg(feature = "dim3")]
mod convex;
#[cfg(feature = "dim2")]
mod convex_polygon;
#[cfg(feature = "dim2")]
mod convex_polygonal_feature2;
#[cfg(feature = "dim3")]
mod convex_polygonal_feature3;
mod convex_polyhedron;
mod cuboid;
#[cfg(feature = "dim3")]
mod cylinder;
mod deformable_shape;
#[cfg(feature = "dim2")]
mod heightfield2;
#[cfg(feature = "dim3")]
mod heightfield3;
mod plane;
mod polyline;
mod segment;
#[doc(hidden)]
pub mod shape;
mod shape_impl;
#[doc(hidden)]
pub mod support_map;
#[cfg(feature = "dim3")]
mod tetrahedron;
mod triangle;
#[cfg(feature = "dim3")]
mod trimesh;
