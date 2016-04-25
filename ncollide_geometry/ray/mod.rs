//! Ray-casting related definitions and implementations.
#[doc(inline)]
pub use ray::ray::{Ray, RayCast, RayIntersection};
pub use ray::ray_plane::plane_toi_with_ray;
pub use ray::ray_triangle::triangle_ray_intersection;
pub use ray::ray_support_map::implicit_toi_and_normal_with_ray;
pub use ray::ray_ball::ball_toi_with_ray;
pub use ray::ray_bvt::{RayIntersectionCostFn, RayInterferencesCollector};

use na::{Point2, Vector2, Point3, Vector3};

#[doc(hidden)]
pub mod ray;
mod ray_plane;
mod ray_ball;
mod ray_cuboid;
mod ray_aabb;
mod ray_bounding_sphere;
mod ray_support_map;
mod ray_triangle;
mod ray_compound;
mod ray_mesh;
mod ray_repr;
mod ray_bvt;

/*
 *
 * Aliases.
 *
 */
/// A 3D ray.
pub type Ray3<N> = Ray<Point3<N>>;

/// A 2D ray.
pub type Ray2<N> = Ray<Point2<N>>;

/// A 3D ray intersection.
pub type RayIntersection3<N> = RayIntersection<Vector3<N>>;

/// A 2D ray intersection.
pub type RayIntersection2<N> = RayIntersection<Vector2<N>>;
