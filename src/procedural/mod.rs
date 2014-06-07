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
pub use procedural::to_trimesh::to_trimesh::ToTriMesh;
// pub use procedural::to_polyline::to_polyline::ToPolyline;


pub mod utils;
pub mod path;
mod trimesh;
mod polyline;

mod to_trimesh {
    pub mod to_trimesh;
    #[cfg(dim3)]
    mod ball_to_trimesh;
    mod plane_to_trimesh;
    #[cfg(dim3)]
    mod bezier_surface_to_trimesh;
    #[cfg(dim3)]
    mod capsule_to_trimesh;
    #[cfg(dim3)]
    mod cone_to_trimesh;
    #[cfg(dim3)]
    mod cuboid_to_trimesh;
    #[cfg(dim3)]
    mod cylinder_to_trimesh;
    mod geom_with_margin_to_trimesh;
    #[cfg(dim3)]
    mod mesh_to_trimesh;
    mod reflection_to_trimesh;
    mod triangle_to_trimesh;
}

// mod to_polyline {
//     pub mod to_polyline;
// }

mod bezier;
mod capsule;
mod cone;
mod cube;
mod cylinder;
mod quad;
mod sphere;
