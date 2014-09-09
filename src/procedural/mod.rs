//! Procedural mesh generation.
pub use procedural::trimesh::{TriMesh, IndexBuffer, UnifiedIndexBuffer, SplitIndexBuffer};
pub use procedural::polyline::Polyline;

pub use procedural::parametric_surface::{parametric_surface_uniform,
                                         parametric_surface_uniform_with_distance_error};
pub use procedural::bezier::{bezier_surface, rational_bezier_surface, bezier_curve, rational_bezier_curve,
                             bezier_surface_at, bezier_curve_at};
pub use procedural::capsule::capsule;
pub use procedural::cone::{unit_cone, cone};
pub use procedural::cuboid::{cuboid, unit_cuboid, rectangle, unit_rectangle};
pub use procedural::cylinder::{unit_cylinder, cylinder};
pub use procedural::quad::{quad, unit_quad, quad_with_vertices};
pub use procedural::sphere::{sphere, unit_sphere, circle, unit_circle};
pub use procedural::convex_hull::{convex_hull3d, convex_hull2d, convex_hull2d_idx};
pub use procedural::to_trimesh::to_trimesh::ToTriMesh;
pub use procedural::to_polyline::to_polyline::ToPolyline;
pub use procedural::hacd::hacd;


pub mod utils;
pub mod path;
mod trimesh;
mod polyline;

mod to_trimesh {
    pub mod to_trimesh;
    mod ball_to_trimesh;
    mod plane_to_trimesh;
    mod bezier_surface_to_trimesh;
    mod capsule_to_trimesh;
    mod cone_to_trimesh;
    mod cuboid_to_trimesh;
    mod cylinder_to_trimesh;
    mod mesh_to_trimesh;
    mod reflection_to_trimesh;
    mod triangle_to_trimesh;
    mod minkowski_sum_to_trimesh;
}

mod to_polyline {
    pub mod to_polyline;
    mod ball_to_polyline;
    mod bezier_curve_to_polyline;
    mod cuboid_to_polyline;
    // FIXME: this one is not easy to implement.
    // #[dim2]
    // mod mesh_to_polyline;
    mod plane_to_polyline;
    mod reflection_to_polyline;
    mod segment_to_polyline;
    mod triangle_to_polyline;
    mod minkowski_sum_to_polyline;
}

mod bezier;
mod capsule;
mod cone;
mod cuboid;
mod cylinder;
mod quad;
mod sphere;
mod convex_hull;
mod hacd;
mod parametric_surface;
