//! Implementation details of the `time_of_impact` function.

pub use self::ball_against_ball::{
    ball_against_ball, ball_against_ball_with_normal,
};
pub use self::composite_shape_against_shape::{
    composite_shape_against_shape, shape_against_composite_shape,
    composite_shape_against_shape_with_normal,
    shape_against_composite_shape_with_normal,
};
pub use self::plane_against_support_map::{
    plane_against_support_map, support_map_against_plane,
    plane_against_support_map_with_normal,
    support_map_against_plane_with_normal,
};
pub use self::shape_against_shape::shape_against_shape as time_of_impact;
pub use self::shape_against_shape::shape_against_shape_with_normal as time_of_impact_and_normal;
pub use self::support_map_against_support_map::{
    support_map_against_support_map,
    support_map_against_support_map_with_normal,
};

mod ball_against_ball;
mod composite_shape_against_shape;
mod plane_against_support_map;
mod shape_against_shape;
mod support_map_against_support_map;
