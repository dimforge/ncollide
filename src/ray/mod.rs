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
pub type Ray3  = Ray<Pnt3<f32>, Vec3<f32>>;
/// 3D ray using double precision.
pub type Ray3d = Ray<Pnt3<f64>, Vec3<f64>>;

/// 2D ray using single precision.
pub type Ray2  = Ray<Pnt2<f32>, Vec2<f32>>;
/// 2D ray using double precision.
pub type Ray2d = Ray<Pnt2<f64>, Vec2<f64>>;

/// 3D ray intersection using single precision.
pub type RayIntersection3  = RayIntersection<f32, Vec3<f32>>;
/// 3D ray intersection using double precision.
pub type RayIntersection3d = RayIntersection<f64, Vec3<f64>>;

/// 2D ray intersection using single precision.
pub type RayIntersection2  = RayIntersection<f32, Vec2<f32>>;
/// 2D ray intersection using double precision.
pub type RayIntersection2d = RayIntersection<f64, Vec2<f64>>;
