//! Bounding volumes.

// Types and traits
#[doc(inline)]
pub use bounding_volume::bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};
#[doc(inline)]
pub use bounding_volume::aabb::{HasAABB, AABB, WithAABB};
pub use bounding_volume::bounding_sphere::{HasBoundingSphere, BoundingSphere};
pub use bounding_volume::spacialized_cone::SpacializedCone;


// // functions
pub use bounding_volume::aabb_utils::{implicit_shape_aabb, point_cloud_aabb};
pub use bounding_volume::aabb_ball::ball_aabb;
pub use bounding_volume::bounding_sphere_utils::{point_cloud_bounding_sphere_with_center, point_cloud_bounding_sphere};
// 
// modules
#[doc(hidden)]
pub mod aabb;
#[doc(hidden)]
pub mod bounding_volume;

mod aabb_box;
mod aabb_cone;
mod aabb_ball;
mod aabb_cylinder;
mod aabb_capsule;
mod aabb_plane;
mod aabb_convex;
mod aabb_compound;
mod aabb_triangle;
mod aabb_segment;
mod aabb_mesh;
mod aabb_bezier_surface;
mod aabb_utils;

mod bounding_sphere;
mod bounding_sphere_box;
mod bounding_sphere_cone;
mod bounding_sphere_ball;
mod bounding_sphere_cylinder;
mod bounding_sphere_capsule;
mod bounding_sphere_plane;
mod bounding_sphere_convex;
mod bounding_sphere_compound;
mod bounding_sphere_triangle;
mod bounding_sphere_segment;
mod bounding_sphere_mesh;
mod bounding_sphere_bezier_surface;
mod bounding_sphere_utils;

mod spacialized_cone;
