//! Implementation details of the `proximity` function.

pub use self::proximity::Proximity;
pub use self::proximity_ball_ball::proximity_ball_ball;
pub use self::proximity_composite_shape_shape::{
    proximity_composite_shape_shape, proximity_shape_composite_shape,
};
pub use self::proximity_plane_support_map::{
    proximity_plane_support_map, proximity_support_map_plane,
};
pub use self::proximity_shape_shape::proximity;
pub use self::proximity_support_map_support_map::proximity_support_map_support_map;
pub use self::proximity_support_map_support_map::proximity_support_map_support_map_with_params;

mod proximity;
mod proximity_ball_ball;
mod proximity_composite_shape_shape;
mod proximity_plane_support_map;
mod proximity_shape_shape;
mod proximity_support_map_support_map;
