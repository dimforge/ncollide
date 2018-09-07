//! Ray-casting related definitions and implementations.

#[doc(inline)]
pub use self::ray::{Ray, RayCast, RayIntersection};
pub use self::ray_ball::ball_toi_with_ray;
pub use self::ray_bvt::RayIntersectionCostFn;
pub use self::ray_plane::{plane_toi_with_line, plane_toi_with_ray};
pub use self::ray_support_map::implicit_toi_and_normal_with_ray;
#[cfg(feature = "dim3")]
pub use self::ray_triangle::triangle_ray_intersection;

#[doc(hidden)]
pub mod ray;
mod ray_plane;
mod ray_ball;
mod ray_cuboid;
mod ray_aabb;
mod ray_bounding_sphere;
mod ray_support_map;
#[cfg(feature = "dim3")]
mod ray_triangle;
mod ray_compound;
#[cfg(feature = "dim3")]
mod ray_trimesh;
mod ray_polyline;
mod ray_shape;
mod ray_bvt;
