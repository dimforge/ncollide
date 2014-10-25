//! Bounding volumes.

#[doc(inline)]
pub use bounding_volume::bounding_volume::{HasBoundingVolume, BoundingVolume};
#[doc(inline)]
pub use bounding_volume::aabb::{HasAABB, AABB};
#[doc(inline)]
pub use bounding_volume::bounding_sphere::{HasBoundingSphere, BoundingSphere};

pub use bounding_volume::aabb_utils::{implicit_shape_aabb, point_cloud_aabb};
pub use bounding_volume::aabb_ball::ball_aabb;
pub use bounding_volume::bounding_sphere_utils::{point_cloud_bounding_sphere_with_center, point_cloud_bounding_sphere};

use na::{Pnt2, Pnt3};

#[doc(hidden)]
pub mod bounding_volume;
#[doc(hidden)]
pub mod aabb;
mod aabb_cuboid;
mod aabb_implicit;
mod aabb_ball;
mod aabb_plane;
mod aabb_convex;
mod aabb_compound;
mod aabb_mesh;
mod aabb_bezier_surface;
mod aabb_utils;

#[doc(hidden)]
pub mod bounding_sphere;
mod bounding_sphere_cuboid;
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

/*
 *
 * Aliases.
 *
 */
/// 2D bounding sphere that uses double precision.
pub type BoundingSphere2d = BoundingSphere<f64, Pnt2<f64>>;
/// 2D AABB that uses double precision.
pub type AABB2d = AABB<Pnt2<f64>>;

/// 3D bounding sphere that uses double precision.
pub type BoundingSphere3d = BoundingSphere<f64, Pnt3<f64>>;
/// 3D AABB that uses double precision.
pub type AABB3d = AABB<Pnt3<f64>>;

/// 2D bounding sphere that uses single precision.
pub type BoundingSphere2 = BoundingSphere<f32, Pnt2<f32>>;
/// 2D AABB that uses single precision.
pub type AABB2 = AABB<Pnt2<f32>>;

/// 3D bounding sphere that uses single precision.
pub type BoundingSphere3 = BoundingSphere<f32, Pnt3<f32>>;
/// 3D AABB that uses single precision.
pub type AABB3 = AABB<Pnt3<f32>>;
