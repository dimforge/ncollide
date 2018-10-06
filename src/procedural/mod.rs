//! Procedural mesh generation.

pub use self::bezier::{bezier_curve, bezier_curve_at};
#[cfg(feature = "dim3")]
pub use self::bezier::{bezier_surface, bezier_surface_at};
#[cfg(feature = "dim3")]
pub use self::capsule::capsule;
#[cfg(feature = "dim3")]
pub use self::cone::{cone, unit_cone};
#[cfg(feature = "dim2")]
pub use self::cuboid::{rectangle, unit_rectangle};
#[cfg(feature = "dim3")]
pub use self::cuboid::{cuboid, unit_cuboid};
#[cfg(feature = "dim3")]
pub use self::cylinder::{cylinder, unit_cylinder};
#[cfg(feature = "dim2")]
pub use self::polyline::Polyline;
#[cfg(feature = "dim3")]
pub use self::quad::{quad, quad_with_vertices, unit_quad};
#[cfg(feature = "dim2")]
pub use self::sphere::{circle, unit_circle};
#[cfg(feature = "dim3")]
pub use self::sphere::{sphere, unit_hemisphere, unit_sphere};
#[cfg(feature = "dim3")]
pub use self::trimesh::{IndexBuffer, TriMesh};

pub mod utils;
#[cfg(feature = "dim3")]
pub mod path;
#[cfg(feature = "dim3")]
mod trimesh;
#[cfg(feature = "dim2")]
mod polyline;

mod sphere;
#[cfg(feature = "dim3")]
mod capsule;
#[cfg(feature = "dim3")]
mod cone;
mod cuboid;
#[cfg(feature = "dim3")]
mod cylinder;
#[cfg(feature = "dim3")]
mod quad;
mod bezier;