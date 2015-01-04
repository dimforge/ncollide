//! Collision shapes supported by ncollide.
#[doc(inline)]
pub use shape::ball::Ball;
pub use shape::plane::Plane;
pub use shape::cuboid::Cuboid;
pub use shape::capsule::Capsule;
pub use shape::cone::Cone;
pub use shape::cylinder::Cylinder;
pub use shape::convex::Convex;
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

use na::{Pnt2, Pnt3, Vec2, Vec3, Iso2, Iso3};

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

/*
 *
 * Aliases.
 *
 */
#[doc = "A 2D ball."] pub type Ball2<N> = Ball<N>;
#[doc = "A 2D plane."] pub type Plane2<N> = Plane<Vec2<N>>;
#[doc = "A 2D cuboid."] pub type Cuboid2<N> = Cuboid<Vec2<N>>;
#[doc = "A 2D capsule."] pub type Capsule2<N> = Capsule<N>;
#[doc = "A 2D cone."] pub type Cone2<N> = Cone<N>;
#[doc = "A 2D cylinder."] pub type Cylinder2<N> = Cylinder<N>;
#[doc = "A 2D convex polytope."] pub type Convex2<N> = Convex<Pnt2<N>>;
#[doc = "A 2D segment."] pub type Segment2<N> = Segment<Pnt2<N>>;
#[doc = "A 2D triangle."] pub type Triangle2<N> = Triangle<Pnt2<N>>;
#[doc = "A 2D polyline."] pub type Polyline2<N> = Polyline<N, Pnt2<N>, Vec2<N>>;
#[doc = "A 2D compound shape."] pub type Compound2<N> = Compound<N, Pnt2<N>, Vec2<N>, Iso2<N>>;

#[doc = "A 3D ball."] pub type Ball3<N> = Ball<N>;
#[doc = "A 3D plane."] pub type Plane3<N> = Plane<Vec3<N>>;
#[doc = "A 3D cuboid."] pub type Cuboid3<N> = Cuboid<Vec3<N>>;
#[doc = "A 3D capsule."] pub type Capsule3<N> = Capsule<N>;
#[doc = "A 3D cone."] pub type Cone3<N> = Cone<N>;
#[doc = "A 3D cylinder."] pub type Cylinder3<N> = Cylinder<N>;
#[doc = "A 3D convex polytope."] pub type Convex3<N> = Convex<Pnt3<N>>;
#[doc = "A 3D segment."] pub type Segment3<N> = Segment<Pnt3<N>>;
#[doc = "A 3D triangle."] pub type Triangle3<N> = Triangle<Pnt3<N>>;
#[doc = "A 3D polyline."] pub type Polyline3<N> = Polyline<N, Pnt3<N>, Vec3<N>>;
#[doc = "A 3D triangle mesh."] pub type TriMesh3<N> = TriMesh<N, Pnt3<N>, Vec3<N>>;
#[doc = "A 3D compound shape."] pub type Compound3<N> = Compound<N, Pnt3<N>, Vec3<N>, Iso3<N>>;
