//! Bounding volumes.

#[doc(inline)]
pub use bounding_volume::bounding_volume::{BoundingVolume, HasBoundingVolume};
#[doc(inline)]
pub use bounding_volume::aabb::{aabb, AABB};
#[doc(inline)]
pub use bounding_volume::bounding_sphere::{bounding_sphere, BoundingSphere};

pub use bounding_volume::aabb_utils::{point_cloud_aabb, support_map_aabb};
pub use bounding_volume::aabb_ball::ball_aabb;
pub use bounding_volume::bounding_sphere_utils::{point_cloud_bounding_sphere,
                                                 point_cloud_bounding_sphere_with_center};
pub use bounding_volume::bounding_volume_bvt::BoundingVolumeInterferencesCollector;

pub use self::polyhedral_cone::PolyhedralCone;

use na::{Point2, Point3};

#[doc(hidden)]
pub mod bounding_volume;
mod bounding_volume_bvt;

#[doc(hidden)]
pub mod aabb;
mod aabb_cuboid;
mod aabb_support_map;
mod aabb_ball;
mod aabb_plane;
mod aabb_convex;
mod aabb_convex_polygon;
mod aabb_compound;
mod aabb_mesh;
mod aabb_utils;
mod aabb_shape;

#[doc(hidden)]
pub mod bounding_sphere;
mod bounding_sphere_cuboid;
mod bounding_sphere_cone;
mod bounding_sphere_ball;
mod bounding_sphere_cylinder;
mod bounding_sphere_capsule;
mod bounding_sphere_plane;
mod bounding_sphere_convex;
mod bounding_sphere_convex_polygon;
mod bounding_sphere_compound;
mod bounding_sphere_triangle;
mod bounding_sphere_segment;
mod bounding_sphere_mesh;
mod bounding_sphere_utils;
mod bounding_sphere_shape;

pub mod polyhedral_cone;

/*
 *
 * Aliases.
 *
 */
/// A 2D bounding sphere.
pub type BoundingSphere2<N> = BoundingSphere<Point2<N>>;
/// A 2D AABB.
pub type AABB2<N> = AABB<Point2<N>>;

/// A 3D bounding sphere:
pub type BoundingSphere3<N> = BoundingSphere<Point3<N>>;
/// A 3D AABB.
pub type AABB3<N> = AABB<Point3<N>>;
