//! Point inclusion and projection.

#[doc(inline)]
pub use self::point_query::{LocalPointQuery, PointQuery};
pub use self::point_bvt::PointInterferencesCollector;

#[doc(hidden)]
pub mod point_query;
mod point_plane;
mod point_ball;
mod point_cuboid;
mod point_aabb;
mod point_bounding_sphere;
mod point_support_map;
mod point_segment;
mod point_triangle;
mod point_compound;
mod point_mesh;
mod point_repr;
mod point_bvt;
