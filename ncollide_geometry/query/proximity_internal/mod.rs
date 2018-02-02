//! Implementation details of the `proximity` function.

pub use self::proximity::Proximity;
pub use self::ball_against_ball::ball_against_ball;
pub use self::support_map_against_support_map::support_map_against_support_map;
pub use self::support_map_against_support_map::support_map_against_support_map_with_params;
pub use self::plane_against_support_map::{plane_against_support_map, support_map_against_plane};
pub use self::shape_against_shape::shape_against_shape as proximity_internal;
pub use self::composite_shape_against_shape::{composite_shape_against_shape,
                                              shape_against_composite_shape};

mod proximity;
mod ball_against_ball;
mod support_map_against_support_map;
mod plane_against_support_map;
mod shape_against_shape;
mod composite_shape_against_shape;
