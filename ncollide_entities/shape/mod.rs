//! Collision shapes supported by ncollide.
#[doc(inline)]
pub use shape::ball::Ball;
pub use shape::plane::Plane;
pub use shape::cuboid::Cuboid;
pub use shape::capsule::Capsule;
pub use shape::cone::Cone;
pub use shape::cylinder::Cylinder;
pub use shape::convex::ConvexHull;
pub use shape::minkowski_sum::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint};
pub use shape::reflection::Reflection;
pub use shape::compound::Compound;
pub use shape::base_mesh::{BaseMesh, BaseMeshElement};
pub use shape::trimesh::TriMesh;
pub use shape::polyline::Polyline;
pub use shape::segment::Segment;
pub use shape::triangle::Triangle;
pub use shape::torus::Torus;
pub use shape::composite_shape::CompositeShape;
pub use shape::shape::ShapeHandle;

use na::{Point2, Point3, Vector2, Vector3, Isometry2, Isometry3};

mod plane;
mod cuboid;
mod minkowski_sum;
mod segment;
mod triangle;
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
mod composite_shape;
mod shape;

/*
 *
 * Aliases.
 *
 */
#[doc = "A 2D ball."] pub type Ball2<N> = Ball<N>;
#[doc = "A 2D plane."] pub type Plane2<N> = Plane<Vector2<N>>;
#[doc = "A 2D cuboid."] pub type Cuboid2<N> = Cuboid<Vector2<N>>;
#[doc = "A 2D capsule."] pub type Capsule2<N> = Capsule<N>;
#[doc = "A 2D cone."] pub type Cone2<N> = Cone<N>;
#[doc = "A 2D cylinder."] pub type Cylinder2<N> = Cylinder<N>;
#[doc = "A 2D convex polytope."] pub type ConvexHull2<N> = ConvexHull<Point2<N>>;
#[doc = "A 2D segment."] pub type Segment2<N> = Segment<Point2<N>>;
#[doc = "A 2D triangle."] pub type Triangle2<N> = Triangle<Point2<N>>;
#[doc = "A 2D polyline."] pub type Polyline2<N> = Polyline<Point2<N>>;
#[doc = "A 2D compound shape."] pub type Compound2<N> = Compound<Point2<N>, Isometry2<N>>;
#[doc = "A 2D shared shape handle."] pub type ShapeHandle2<N> = ShapeHandle<Point2<N>, Isometry2<N>>;

#[doc = "A 3D ball."] pub type Ball3<N> = Ball<N>;
#[doc = "A 3D plane."] pub type Plane3<N> = Plane<Vector3<N>>;
#[doc = "A 3D cuboid."] pub type Cuboid3<N> = Cuboid<Vector3<N>>;
#[doc = "A 3D capsule."] pub type Capsule3<N> = Capsule<N>;
#[doc = "A 3D cone."] pub type Cone3<N> = Cone<N>;
#[doc = "A 3D cylinder."] pub type Cylinder3<N> = Cylinder<N>;
#[doc = "A 3D convex polytope."] pub type ConvexHull3<N> = ConvexHull<Point3<N>>;
#[doc = "A 3D segment."] pub type Segment3<N> = Segment<Point3<N>>;
#[doc = "A 3D triangle."] pub type Triangle3<N> = Triangle<Point3<N>>;
#[doc = "A 3D polyline."] pub type Polyline3<N> = Polyline<Point3<N>>;
#[doc = "A 3D triangle mesh."] pub type TriMesh3<N> = TriMesh<Point3<N>>;
#[doc = "A 3D compound shape."] pub type Compound3<N> = Compound<Point3<N>, Isometry3<N>>;
#[doc = "A 3D shared shape handle."] pub type ShapeHandle3<N> = ShapeHandle<Point3<N>, Isometry3<N>>;
