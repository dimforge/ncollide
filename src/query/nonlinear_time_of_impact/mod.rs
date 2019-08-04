//! Implementation details of the `nonlinear_time_of_impact` function.

pub use self::nonlinear_time_of_impact_ball_ball::nonlinear_time_of_impact_ball_ball;
pub use self::nonlinear_time_of_impact_composite_shape_shape::{
    nonlinear_time_of_impact_composite_shape_shape, nonlinear_time_of_impact_shape_composite_shape,
};
//pub use self::nonlinear_time_of_impact_plane_support_map::{nonlinear_time_of_impact_plane_support_map, nonlinear_time_of_impact_support_map_plane};
pub use self::nonlinear_time_of_impact::nonlinear_time_of_impact;
pub use self::nonlinear_time_of_impact_support_map_support_map::{
    nonlinear_time_of_impact_support_map_support_map, nonlinear_time_of_impact_support_map_support_map_with_closest_points_function
};


mod nonlinear_time_of_impact_ball_ball;
mod nonlinear_time_of_impact_composite_shape_shape;
//mod nonlinear_time_of_impact_plane_support_map;
mod nonlinear_time_of_impact;
mod nonlinear_time_of_impact_support_map_support_map;
