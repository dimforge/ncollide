//! Geometric primitives.

// traits and types
pub use geom::ball::Ball;
pub use geom::plane::Plane;
pub use geom::box_geom::Box;
pub use geom::capsule::Capsule;
pub use geom::cone::Cone;
pub use geom::cylinder::Cylinder;
pub use geom::convex::Convex;
pub use geom::minkowski_sum::{MinkowskiSum, AnnotatedMinkowskiSum, AnnotatedPoint};
pub use geom::reflection::Reflection;
pub use geom::compound::Compound;
pub use geom::geom_with_margin::GeomWithMargin;
pub use geom::mesh::{Mesh, MeshElement};
pub use geom::segment::Segment;
pub use geom::triangle::Triangle;
pub use geom::geom::{Geom, ConcaveGeom};

// modules
pub mod geom;
pub mod ball;
pub mod plane;
pub mod box_geom;
pub mod capsule;
pub mod cone;
pub mod cylinder;
pub mod convex;
pub mod minkowski_sum;
pub mod reflection;
pub mod compound;
pub mod segment;
pub mod triangle;
pub mod geom_with_margin;
pub mod mesh;
