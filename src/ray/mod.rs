//! Ray casting utilities.

// types an traits
pub use ray::ray::{Ray, RayCast, RayCastWithTransform};

// functions
pub use ray::ray_plane::plane_toi_with_ray;
pub use ray::ray_implicit::gjk_toi_and_normal_with_ray;
pub use ray::ray_ball::ball_toi_with_ray;

#[cfg(dim3)]
pub use ray::ray_triangle::triangle_ray_intersection;

// modules
pub mod ray;
#[doc(hidden)]
pub mod ray_plane;
#[doc(hidden)]
pub mod ray_ball;
#[doc(hidden)]
pub mod ray_box;
#[doc(hidden)]
pub mod ray_aabb;
#[doc(hidden)]
pub mod ray_implicit;
#[doc(hidden)]
pub mod ray_triangle;
#[doc(hidden)]
pub mod ray_concave;
#[doc(hidden)]
pub mod ray_mesh;
