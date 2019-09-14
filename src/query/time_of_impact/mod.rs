//! Implementation details of the `time_of_impact` function.

pub use self::time_of_impact::{time_of_impact, TOIStatus, TOI};
pub use self::time_of_impact_ball_ball::time_of_impact_ball_ball;
pub use self::time_of_impact_composite_shape_shape::{
    time_of_impact_composite_shape_shape, time_of_impact_shape_composite_shape,
};
pub use self::time_of_impact_plane_support_map::{
    time_of_impact_plane_support_map, time_of_impact_support_map_plane,
};
pub use self::time_of_impact_support_map_support_map::time_of_impact_support_map_support_map;

mod time_of_impact;
mod time_of_impact_ball_ball;
mod time_of_impact_composite_shape_shape;
mod time_of_impact_plane_support_map;
mod time_of_impact_support_map_support_map;
