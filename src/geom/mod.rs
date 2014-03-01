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
pub use geom::mesh::{Mesh, MeshElement, MeshPrimitive};
pub use geom::segment::Segment;
pub use geom::triangle::Triangle;
pub use geom::geom::{Geom, ConcaveGeom};

// modules
mod geom;
mod ball;
mod plane;
mod box_geom;
mod capsule;
mod cone;
mod cylinder;
mod convex;
mod minkowski_sum;
mod reflection;
mod compound;
mod segment;
mod triangle;
mod geom_with_margin;
mod mesh;
