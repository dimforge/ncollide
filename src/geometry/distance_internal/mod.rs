//! Implementation details of the `distance` function.

pub use self::ball_against_ball::ball_against_ball;
pub use self::support_map_against_support_map::support_map_against_support_map;
pub use self::support_map_against_support_map::support_map_against_support_map_with_params;
pub use self::plane_against_support_map::{plane_against_support_map, support_map_against_plane};
pub use self::shape_against_shape::shape_against_shape;
pub use self::concave_shape_against_shape::{concave_shape_against_shape, shape_against_concave_shape};
pub use self::distance_with::DistanceTo;

#[path = "../dispatch_utils.rs"]
mod dispatch_utils;

mod ball_against_ball;
mod support_map_against_support_map;
mod plane_against_support_map;
mod shape_against_shape;
mod concave_shape_against_shape;

pub mod distance_with;
