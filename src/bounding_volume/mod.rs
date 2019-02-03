//! Bounding volumes.

pub use self::circular_cone::CircularCone;
pub use self::spatialized_normal_cone::SpatializedNormalCone;
#[doc(inline)]
pub use crate::bounding_volume::aabb::{aabb, AABB};
pub use crate::bounding_volume::aabb_ball::ball_aabb;
pub use crate::bounding_volume::aabb_utils::{point_cloud_aabb, support_map_aabb};
#[doc(inline)]
pub use crate::bounding_volume::bounding_sphere::{bounding_sphere, BoundingSphere};
pub use crate::bounding_volume::bounding_sphere_utils::{
    point_cloud_bounding_sphere, point_cloud_bounding_sphere_with_center,
};
#[doc(inline)]
pub use crate::bounding_volume::bounding_volume::{BoundingVolume, HasBoundingVolume};

#[doc(hidden)]
pub mod bounding_volume;

#[doc(hidden)]
pub mod aabb;
mod aabb_ball;
mod aabb_compound;
#[cfg(feature = "dim3")]
mod aabb_convex;
#[cfg(feature = "dim2")]
mod aabb_convex_polygon;
mod aabb_cuboid;
mod aabb_plane;
mod aabb_polyline;
mod aabb_shape;
mod aabb_support_map;
#[cfg(feature = "dim3")]
mod aabb_trimesh;
mod aabb_heightfield;
mod aabb_utils;

#[doc(hidden)]
pub mod bounding_sphere;
mod bounding_sphere_ball;
mod bounding_sphere_capsule;
mod bounding_sphere_compound;
#[cfg(feature = "dim3")]
mod bounding_sphere_cone;
#[cfg(feature = "dim3")]
mod bounding_sphere_convex;
#[cfg(feature = "dim2")]
mod bounding_sphere_convex_polygon;
mod bounding_sphere_cuboid;
#[cfg(feature = "dim3")]
mod bounding_sphere_cylinder;
mod bounding_sphere_plane;
mod bounding_sphere_polyline;
mod bounding_sphere_segment;
mod bounding_sphere_shape;
#[cfg(feature = "dim3")]
mod bounding_sphere_triangle;
#[cfg(feature = "dim3")]
mod bounding_sphere_trimesh;
mod bounding_sphere_heightfield;
mod bounding_sphere_utils;

pub(crate) mod circular_cone;
mod spatialized_normal_cone;
