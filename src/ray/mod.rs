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
mod ray;
mod ray_plane;
mod ray_ball;
mod ray_box;
mod ray_aabb;
mod ray_implicit;
mod ray_triangle;
mod ray_concave;
mod ray_mesh;
mod ray_bvt;
