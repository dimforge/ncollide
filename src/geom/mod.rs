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
mod geom;
mod implicit;
mod ball;
mod plane;
mod box;
mod capsule;
mod cone;
mod cylinder;
mod convex_polytope;
mod minkowski_sum;
mod reflection;
mod compound;
mod geom_with_margin;
