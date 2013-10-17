//! Geometric primitives.

// traits and types
pub use geom::implicit::{Implicit, HasMargin};
pub use geom::ball::Ball;
pub use geom::plane::Plane;
pub use geom::box::Box;
pub use geom::capsule::Capsule;
pub use geom::cone::Cone;
pub use geom::cylinder::Cylinder;
pub use geom::convex_polytope::ConvexPolytope;
pub use geom::minkowski_sum::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint};
pub use geom::reflection::Reflection;
pub use geom::compound::CompoundAABB;
pub use geom::geom_with_margin::GeomWithMargin;
pub use geom::geom::{Geom, IGeom, PlaneGeom, BallGeom, BoxGeom, ConeGeom, CylinderGeom,
                     CapsuleGeom, CompoundGeom, ImplicitGeom};


// methods
pub use geom::minkowski_sum::cso_support_point;
pub use geom::minkowski_sum::cso_support_point_without_margin;

// modules
#[doc(hidden)]
pub mod geom;
#[doc(hidden)]
pub mod implicit;
#[doc(hidden)]
pub mod ball;
#[doc(hidden)]
pub mod plane;
#[doc(hidden)]
pub mod box;
#[doc(hidden)]
pub mod capsule;
#[doc(hidden)]
pub mod cone;
#[doc(hidden)]
pub mod cylinder;
#[doc(hidden)]
pub mod convex_polytope;
#[doc(hidden)]
pub mod minkowski_sum;
#[doc(hidden)]
pub mod reflection;
#[doc(hidden)]
pub mod compound;
#[doc(hidden)]
pub mod geom_with_margin;
