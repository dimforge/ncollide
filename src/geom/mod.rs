//! Geometric primitives.

// traits and types
pub use geom::ball::Ball;
pub use geom::plane::Plane;
pub use geom::cuboid::Cuboid;
pub use geom::capsule::Capsule;
pub use geom::cone::Cone;
pub use geom::cylinder::Cylinder;
pub use geom::convex::Convex;
pub use geom::minkowski_sum::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint};
pub use geom::reflection::Reflection;
pub use geom::compound::{Compound, CompoundData};
pub use geom::mesh::{Mesh, MeshElement, MeshPrimitive};
pub use geom::segment::Segment;
pub use geom::triangle::Triangle;
#[doc(inline)]
pub use geom::geom::{Geom, ConcaveGeom};
pub use geom::bezier_surface::{BezierSurface, BezierSurfaceEvaluationCache};
pub use geom::bezier_curve::{BezierCurve, BezierCurveEvaluationCache};
pub use geom::torus::Torus;
// pub use geom::reparametrized_surface::ReparametrizedSurface;

// modules
#[doc(hidden)]
pub mod geom;
mod ball;
mod plane;
mod cuboid;
mod capsule;
mod cone;
mod cylinder;
mod convex;
mod minkowski_sum;
mod reflection;
mod compound;
mod segment;
mod triangle;
mod mesh;
mod bezier_curve;
mod bezier_surface;
mod torus;
// mod reparametrized_surface;
