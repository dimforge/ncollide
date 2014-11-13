//! Collision shapes supported by ncollide.
#[doc(inline)]
pub use shape::shape::{Shape, ConcaveShape};
pub use shape::ball::Ball;
pub use shape::plane::Plane;
pub use shape::cuboid::Cuboid;
pub use shape::capsule::Capsule;
pub use shape::cone::Cone;
pub use shape::cylinder::Cylinder;
pub use shape::convex::Convex;
pub use shape::minkowski_sum::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint};
pub use shape::reflection::Reflection;
pub use shape::compound::{Compound, CompoundData};
pub use shape::mesh::{Mesh, MeshElement};
pub use shape::segment::Segment;
pub use shape::triangle::Triangle;
pub use shape::bezier_curve::BezierCurve;
pub use shape::bezier_surface::BezierSurface;
pub use shape::torus::Torus;

use na::{Pnt2, Pnt3, Vec2, Vec3, Iso2, Iso3};

#[doc(hidden)]
pub mod shape;
mod plane;
mod cuboid;
mod minkowski_sum;
mod segment;
mod triangle;
mod mesh;
mod bezier_curve;
mod bezier_surface;
mod ball;
mod capsule;
mod cone;
mod cylinder;
mod reflection;
mod torus;
mod compound;
mod convex;

/*
 *
 * Aliases.
 *
 */
#[doc = "2D Shape trait object."]
pub type Shape2<N> = Shape<N, Pnt2<N>, Vec2<N>, Iso2<N>> + Send + Sync;

#[doc = "3D Shape trait object."]
pub type Shape3<N> = Shape<N, Pnt3<N>, Vec3<N>, Iso3<N>> + Send + Sync;

#[doc = "A 2D ball."] pub type Ball2<N> = Ball<N>;
#[doc = "A 2D plane."] pub type Plane2<N> = Plane<Vec2<N>>;
#[doc = "A 2D cuboid."] pub type Cuboid2<N> = Cuboid<Vec2<N>>;
#[doc = "A 2D capsule."] pub type Capsule2<N> = Capsule<N>;
#[doc = "A 2D cone."] pub type Cone2<N> = Cone<N>;
#[doc = "A 2D cylinder."] pub type Cylinder2<N> = Cylinder<N>;
#[doc = "A 2D convex polytope."] pub type Convex2<N> = Convex<Pnt2<N>>;
#[doc = "A 2D segment."] pub type Segment2<N> = Segment<Pnt2<N>>;
#[doc = "A 2D triangle."] pub type Triangle2<N> = Triangle<Pnt2<N>>;
#[doc = "A 2D Bézier curve."] pub type BezierCurve2<N> = BezierCurve<Pnt2<N>>;
#[doc = "A 2D Bézier surface."] pub type BezierSurface2<N> = BezierSurface<Pnt2<N>>;
#[doc = "A 2D Mesh."] pub type Mesh2<N> = Mesh<N, Pnt2<N>, Vec2<N>, Segment2<N>>;
#[doc = "A 2D Compound shape."] pub type Compound2<N> = Compound<N, Pnt2<N>, Vec2<N>, Iso2<N>>;

#[doc = "A 3D ball."] pub type Ball3<N> = Ball<N>;
#[doc = "A 3D plane."] pub type Plane3<N> = Plane<Vec3<N>>;
#[doc = "A 3D cuboid."] pub type Cuboid3<N> = Cuboid<Vec3<N>>;
#[doc = "A 3D capsule."] pub type Capsule3<N> = Capsule<N>;
#[doc = "A 3D cone."] pub type Cone3<N> = Cone<N>;
#[doc = "A 3D cylinder."] pub type Cylinder3<N> = Cylinder<N>;
#[doc = "A 3D convex polytope."] pub type Convex3<N> = Convex<Pnt3<N>>;
#[doc = "A 3D segment."] pub type Segment3<N> = Segment<Pnt3<N>>;
#[doc = "A 3D triangle."] pub type Triangle3<N> = Triangle<Pnt3<N>>;
#[doc = "A 3D Bézier curve."] pub type BezierCurve3<N> = BezierCurve<Pnt3<N>>;
#[doc = "A 3D Bézier surface."] pub type BezierSurface3<N> = BezierSurface<Pnt3<N>>;
#[doc = "A 3D Mesh."] pub type Mesh3<N> = Mesh<N, Pnt3<N>, Vec3<N>, Triangle3<N>>;
#[doc = "A 3D Compound shape."] pub type Compound3<N> = Compound<N, Pnt3<N>, Vec3<N>, Iso3<N>>;
