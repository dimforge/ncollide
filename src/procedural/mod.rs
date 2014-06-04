//! Procedural mesh generation.
pub use procedural::trimesh::{TriMesh, IndexBuffer, UnifiedIndexBuffer, SplitIndexBuffer};
pub use procedural::polyline::Polyline;

pub use procedural::bezier::{bezier_surface, rational_bezier_surface, bezier_curve, rational_bezier_curve,
                             bezier_surface_at, bezier_curve_at};
pub use procedural::capsule::capsule;
pub use procedural::cone::{unit_cone, cone};
pub use procedural::cube::{cube, unit_cube};
pub use procedural::cylinder::{unit_cylinder, cylinder};
pub use procedural::quad::{quad, unit_quad, quad_with_vertices};
pub use procedural::sphere::{sphere, unit_sphere, circle, unit_circle};


pub mod utils;
pub mod path;
mod trimesh;
mod polyline;

mod bezier;
mod capsule;
mod cone;
mod cube;
mod cylinder;
mod quad;
mod sphere;
