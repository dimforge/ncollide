//! Collision shapes supported by ncollide.

pub use self::ball::Ball;
pub use self::plane::Plane;
pub use self::cuboid::Cuboid;
pub use self::capsule::Capsule;
pub use self::cone::Cone;
pub use self::cylinder::Cylinder;
pub use self::convex::ConvexHull;
pub use self::minkowski_sum::{cso_support_point, AnnotatedCSO, AnnotatedMinkowskiSum,
                              AnnotatedPoint, MinkowskiSum, CSO};
pub use self::reflection::Reflection;
pub use self::compound::Compound;
pub use self::base_mesh::{BaseMesh, BaseMeshElement};
pub use self::trimesh::TriMesh;
pub use self::polyline::Polyline;
pub use self::segment::{Segment, SegmentPointLocation};
pub use self::triangle::{Triangle, TrianglePointLocation};
pub use self::tetrahedron::{Tetrahedron, TetrahedronPointLocation};
pub use self::torus::Torus;
pub use self::convex_polyface::{ConvexPolyface, FeatureId};
#[doc(inline)]
pub use self::composite_shape::CompositeShape;
#[doc(inline)]
pub use self::support_map::SupportMap;
pub use self::convex_polyhedron::ConvexPolyhedron;
#[doc(inline)]
pub use self::shape::{Shape, ShapeHandle};

use na::{Isometry2, Isometry3, Point2, Point3, Vector2, Vector3};

#[doc(hidden)]
pub mod composite_shape;
#[doc(hidden)]
pub mod support_map;
mod convex_polyhedron;
#[doc(hidden)]
pub mod shape;
mod plane;
mod cuboid;
mod minkowski_sum;
mod segment;
mod triangle;
mod tetrahedron;
mod base_mesh;
mod trimesh;
mod polyline;
mod ball;
mod capsule;
mod cone;
mod cylinder;
mod reflection;
mod torus;
mod compound;
mod convex;
mod convex_polyface;
mod shape_impl;

/*
 *
 * Aliases.
 *
 */
#[doc = "A 2D ball."]
pub type Ball2<N> = Ball<N>;
#[doc = "A 2D plane."]
pub type Plane2<N> = Plane<Vector2<N>>;
#[doc = "A 2D cuboid."]
pub type Cuboid2<N> = Cuboid<Vector2<N>>;
#[doc = "A 2D capsule."]
pub type Capsule2<N> = Capsule<N>;
#[doc = "A 2D cone."]
pub type Cone2<N> = Cone<N>;
#[doc = "A 2D cylinder."]
pub type Cylinder2<N> = Cylinder<N>;
#[doc = "A 2D convex polytope."]
pub type ConvexHull2<N> = ConvexHull<Point2<N>>;
#[doc = "A 2D segment."]
pub type Segment2<N> = Segment<Point2<N>>;
#[doc = "A 2D triangle."]
pub type Triangle2<N> = Triangle<Point2<N>>;
#[doc = "A 2D polyline."]
pub type Polyline2<N> = Polyline<Point2<N>>;
#[doc = "A 2D compound shape."]
pub type Compound2<N> = Compound<Point2<N>, Isometry2<N>>;
#[doc = "A 2D abstract composite shape."]
pub type CompositeShape2<N> = CompositeShape<Point2<N>, Isometry2<N>>;
#[doc = "A 2D abstract support mapping."]
pub type SupportMap2<N> = SupportMap<Point2<N>, Isometry2<N>>;
#[doc = "A 2D dynamic shape."]
pub type Shape2<N> = Shape<Point2<N>, Isometry2<N>>;
#[doc = "A 2D shared dynamic shape handle."]
pub type ShapeHandle2<N> = ShapeHandle<Point2<N>, Isometry2<N>>;

#[doc = "A 3D ball."]
pub type Ball3<N> = Ball<N>;
#[doc = "A 3D plane."]
pub type Plane3<N> = Plane<Vector3<N>>;
#[doc = "A 3D cuboid."]
pub type Cuboid3<N> = Cuboid<Vector3<N>>;
#[doc = "A 3D capsule."]
pub type Capsule3<N> = Capsule<N>;
#[doc = "A 3D cone."]
pub type Cone3<N> = Cone<N>;
#[doc = "A 3D cylinder."]
pub type Cylinder3<N> = Cylinder<N>;
#[doc = "A 3D convex polytope."]
pub type ConvexHull3<N> = ConvexHull<Point3<N>>;
#[doc = "A 3D segment."]
pub type Segment3<N> = Segment<Point3<N>>;
#[doc = "A 3D triangle."]
pub type Triangle3<N> = Triangle<Point3<N>>;
#[doc = "A 3D tetrahedron."]
pub type Tetrahedron3<N> = Tetrahedron<Point3<N>>;
#[doc = "A 3D polyline."]
pub type Polyline3<N> = Polyline<Point3<N>>;
#[doc = "A 3D triangle mesh."]
pub type TriMesh3<N> = TriMesh<Point3<N>>;
#[doc = "A 3D compound shape."]
pub type Compound3<N> = Compound<Point3<N>, Isometry3<N>>;
#[doc = "A 3D abstract composite shape."]
pub type CompositeShape3<N> = CompositeShape<Point3<N>, Isometry3<N>>;
#[doc = "A 3D abstract support mapping."]
pub type SupportMap3<N> = SupportMap<Point3<N>, Isometry3<N>>;
#[doc = "A 3D dynamic shape."]
pub type Shape3<N> = Shape<Point3<N>, Isometry3<N>>;
#[doc = "A 3D shared dynamic shape handle."]
pub type ShapeHandle3<N> = ShapeHandle<Point3<N>, Isometry3<N>>;
