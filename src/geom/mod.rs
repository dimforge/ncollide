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
// pub use geom::simplex_mesh::SimplexMesh;
pub use geom::geom::{Geom, IGeom, PlaneGeom, BallGeom, BoxGeom, ConeGeom, CylinderGeom,
                     CapsuleGeom, CompoundGeom, ImplicitGeom};


// methods
pub use geom::minkowski_sum::cso_support_point;
pub use geom::minkowski_sum::cso_support_point_without_margin;

// modules
pub mod geom;
pub mod implicit;
pub mod ball;
pub mod plane;
pub mod box;
pub mod capsule;
pub mod cone;
pub mod cylinder;
pub mod convex_polytope;
pub mod minkowski_sum;
pub mod reflection;
pub mod compound;
pub mod geom_with_margin;
// pub mod simplex_mesh;
