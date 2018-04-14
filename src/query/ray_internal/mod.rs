//! Ray-casting related definitions and implementations.

#[doc(inline)]
pub use self::ray::{Ray, RayCast, RayIntersection};
// pub use self::ray_plane::{plane_toi_with_line, plane_toi_with_ray};
// pub use self::ray_triangle::triangle_ray_intersection;
// pub use self::ray_support_map::implicit_toi_and_normal_with_ray;
// pub use self::ray_ball::ball_toi_with_ray;
pub use self::ray_bvt::{RayInterferencesCollector, RayIntersectionCostFn};

#[doc(hidden)]
pub mod ray;
// mod ray_plane;
// mod ray_ball;
// mod ray_cuboid;
// mod ray_aabb;
// mod ray_bounding_sphere;
// mod ray_support_map;
// mod ray_triangle;
// mod ray_compound;
// mod ray_mesh;
// mod ray_shape;
mod ray_bvt;
