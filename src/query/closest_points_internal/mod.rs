//! Implementation details of the `closest_points` function.

pub use self::closest_points::ClosestPoints;
pub use self::ball_against_ball::ball_against_ball;
pub use self::segment_against_segment::{segment_against_segment,
                                        segment_against_segment_with_locations};
pub use self::line_against_line::line_against_line;
pub use self::support_map_against_support_map::support_map_against_support_map;
pub use self::support_map_against_support_map::support_map_against_support_map_with_params;
pub use self::plane_against_support_map::{plane_against_support_map, support_map_against_plane};
// pub use self::shape_against_shape::shape_against_shape as closest_points_internal;
// pub use self::composite_shape_against_shape::{composite_shape_against_shape,
//                                               shape_against_composite_shape};

mod closest_points;
mod ball_against_ball;
mod segment_against_segment;
mod line_against_line;
mod support_map_against_support_map;
mod plane_against_support_map;
// mod shape_against_shape;
// mod composite_shape_against_shape;
