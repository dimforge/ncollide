//! Ray-casting related definitions and implementations.

#[doc(inline)]
pub use self::ray::{Ray, RayCast, RayIntersection};
pub use self::ray_ball::ray_toi_with_ball;
pub use self::ray_plane::{line_toi_with_plane, ray_toi_with_plane};
pub use self::ray_support_map::ray_intersection_with_support_map_with_params;
#[cfg(feature = "dim3")]
pub use self::ray_triangle::ray_intersection_with_triangle;

#[doc(hidden)]
pub mod ray;
mod ray_aabb;
mod ray_ball;
mod ray_multiball;
mod ray_bounding_sphere;
mod ray_compound;
mod ray_cuboid;
mod ray_plane;
mod ray_polyline;
mod ray_shape;
mod ray_support_map;
#[cfg(feature = "dim3")]
mod ray_triangle;
#[cfg(feature = "dim3")]
mod ray_trimesh;
mod ray_heightfield;
