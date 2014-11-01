//! Collision geometries supported by ncollide.
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

use na::{Pnt2, Pnt3, Vec2, Vec3, Iso2, Iso3, Mat1, Mat3};

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
#[doc = "2D Shape trait object that uses double precision."]
pub type Shape2d = Shape<f64, Pnt2<f64>, Vec2<f64>, Iso2<f64>> + Send + Sync;
#[doc = "2D Shape trait object that uses single precision."]
pub type Shape2 = Shape<f32, Pnt2<f32>, Vec2<f32>, Iso2<f32>> + Send + Sync;

#[doc = "3D Shape trait object that uses double precision."]
pub type Shape3d = Shape<f64, Pnt3<f64>, Vec3<f64>, Iso3<f64>> + Send + Sync;
#[doc = "3D Shape trait object that uses single precision."]
pub type Shape3 = Shape<f32, Pnt3<f32>, Vec3<f32>, Iso3<f32>> + Send + Sync;

#[doc = "2D ball that uses double precision."] pub type Ball2d = Ball<f64>;
#[doc = "2D plane that uses double precision."] pub type Plane2d = Plane<Vec2<f64>>;
#[doc = "2D cuboid that uses double precision."] pub type Cuboid2d = Cuboid<Vec2<f64>>;
#[doc = "2D capsule that uses double precision."] pub type Capsule2d = Capsule<f64>;
#[doc = "2D cone that uses double precision."] pub type Cone2d = Cone<f64>;
#[doc = "2D cylinder that uses double precision."] pub type Cylinder2d = Cylinder<f64>;
#[doc = "2D convex polytope that uses double precision."] pub type Convex2d = Convex<Pnt2<f64>>;
#[doc = "2D segment that uses double precision."] pub type Segment2d = Segment<Pnt2<f64>>;
#[doc = "2D triangle that uses double precision."] pub type Triangle2d = Triangle<Pnt2<f64>>;
#[doc = "2D Bézier curve that uses double precision."] pub type BezierCurve2d = BezierCurve<Pnt2<f64>>;
#[doc = "2D Bézier surface that uses double precision."] pub type BezierSurface2d = BezierSurface<Pnt2<f64>>;
#[doc = "2D Mesh that uses double precision."] pub type Mesh2d = Mesh<f64, Pnt2<f64>, Vec2<f64>, Segment2d>;
#[doc = "2D Compound shape that uses double precision."] pub type Compound2d = Compound<f64, Pnt2<f64>, Vec2<f64>, Iso2<f64>, Mat1<f64>>;

#[doc = "2D ball that uses single precision."] pub type Ball2 = Ball<f32>;
#[doc = "2D plane that uses single precision."] pub type Plane2 = Plane<Vec2<f32>>;
#[doc = "2D cuboid that uses single precision."] pub type Cuboid2 = Cuboid<Vec2<f32>>;
#[doc = "2D capsule that uses single precision."] pub type Capsule2 = Capsule<f32>;
#[doc = "2D cone that uses single precision."] pub type Cone2 = Cone<f32>;
#[doc = "2D cylinder that uses single precision."] pub type Cylinder2 = Cylinder<f32>;
#[doc = "2D convex polytope that uses single precision."] pub type Convex2 = Convex<Pnt2<f32>>;
#[doc = "2D segment that uses single precision."] pub type Segment2 = Segment<Pnt2<f32>>;
#[doc = "2D triangle that uses single precision."] pub type Triangle2 = Triangle<Pnt2<f32>>;
#[doc = "2D Bézier curve that uses single precision."] pub type BezierCurve2 = BezierCurve<Pnt2<f32>>;
#[doc = "2D Bézier surface that uses single precision."] pub type BezierSurface2 = BezierSurface<Pnt2<f32>>;
#[doc = "2D Mesh that uses single precision."] pub type Mesh2 = Mesh<f32, Pnt2<f32>, Vec2<f32>, Segment2>;
#[doc = "2D Compound shape that uses single precision."] pub type Compound2 = Compound<f32, Pnt2<f32>, Vec2<f32>, Iso2<f32>, Mat1<f32>>;

#[doc = "3D ball that uses double precision."] pub type Ball3d = Ball<f64>;
#[doc = "3D plane that uses double precision."] pub type Plane3d = Plane<Vec3<f64>>;
#[doc = "3D cuboid that uses double precision."] pub type Cuboid3d = Cuboid<Vec3<f64>>;
#[doc = "3D capsule that uses double precision."] pub type Capsule3d = Capsule<f64>;
#[doc = "3D cone that uses double precision."] pub type Cone3d = Cone<f64>;
#[doc = "3D cylinder that uses double precision."] pub type Cylinder3d = Cylinder<f64>;
#[doc = "3D convex polytope that uses double precision."] pub type Convex3d = Convex<Pnt3<f64>>;
#[doc = "3D segment that uses double precision."] pub type Segment3d = Segment<Pnt3<f64>>;
#[doc = "3D triangle that uses double precision."] pub type Triangle3d = Triangle<Pnt3<f64>>;
#[doc = "3D Bézier curve that uses double precision."] pub type BezierCurve3d = BezierCurve<Pnt3<f64>>;
#[doc = "3D Bézier surface that uses double precision."] pub type BezierSurface3d = BezierSurface<Pnt3<f64>>;
#[doc = "3D Mesh that uses double precision."] pub type Mesh3d = Mesh<f64, Pnt3<f64>, Vec3<f64>, Triangle3d>;
#[doc = "3D Compound shape that uses double precision."] pub type Compound3d = Compound<f64, Pnt3<f64>, Vec3<f64>, Iso3<f64>, Mat3<f64>>;

#[doc = "3D ball that uses single precision."] pub type Ball3 = Ball<f32>;
#[doc = "3D plane that uses single precision."] pub type Plane3 = Plane<Vec3<f32>>;
#[doc = "3D cuboid that uses single precision."] pub type Cuboid3 = Cuboid<Vec3<f32>>;
#[doc = "3D capsule that uses single precision."] pub type Capsule3 = Capsule<f32>;
#[doc = "3D cone that uses single precision."] pub type Cone3 = Cone<f32>;
#[doc = "3D cylinder that uses single precision."] pub type Cylinder3 = Cylinder<f32>;
#[doc = "3D convex polytope that uses single precision."] pub type Convex3 = Convex<Pnt3<f32>>;
#[doc = "3D segment that uses single precision."] pub type Segment3 = Segment<Pnt3<f32>>;
#[doc = "3D triangle that uses single precision."] pub type Triangle3 = Triangle<Pnt3<f32>>;
#[doc = "3D Bézier curve that uses single precision."] pub type BezierCurve3 = BezierCurve<Pnt3<f32>>;
#[doc = "3D Bézier surface that uses single precision."] pub type BezierSurface3 = BezierSurface<Pnt3<f32>>;
#[doc = "3D Mesh that uses single precision."] pub type Mesh3 = Mesh<f32, Pnt3<f32>, Vec3<f32>, Triangle3>;
#[doc = "3D Compound shape that uses single precision."] pub type Compound3 = Compound<f32, Pnt3<f32>, Vec3<f32>, Iso3<f32>, Mat3<f32>>;
