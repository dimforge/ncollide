//! Ray-casting related definitions and implementations.
#[doc(inline)]
pub use ray::ray::{Ray, LocalRayCast, RayCast, RayIntersection};
pub use ray::ray_plane::plane_toi_with_ray;
pub use ray::ray_triangle::triangle_ray_intersection;
pub use ray::ray_support_map::implicit_toi_and_normal_with_ray;
pub use ray::ray_ball::ball_toi_with_ray;

use na::{Pnt2, Vec2, Pnt3, Vec3};

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
mod ray_bvt;
mod ray_bezier_surface;
mod ray_bezier_curve;

/*
 *
 * Aliases.
 *
 */
/// 3D ray using single precision.
pub type Ray3<N> = Ray<Pnt3<N>, Vec3<N>>;

/// 2D ray using single precision.
pub type Ray2<N> = Ray<Pnt2<N>, Vec2<N>>;

/// 3D ray intersection using single precision.
pub type RayIntersection3<N> = RayIntersection<N, Vec3<N>>;

/// 2D ray intersection using single precision.
pub type RayIntersection2<N> = RayIntersection<N, Vec2<N>>;
