//! Implementation details of the `distance` function.

pub use self::ball_against_ball::ball_against_ball;
pub use self::support_map_against_support_map::support_map_against_support_map;
pub use self::plane_against_support_map::{plane_against_support_map, support_map_against_plane};
pub use self::any_against_any::any_against_any;
pub use self::any_against_any::any_against_any as time_of_impact;
pub use self::composite_shape_against_any::{composite_shape_against_any, any_against_composite_shape};

mod ball_against_ball;
mod support_map_against_support_map;
mod plane_against_support_map;
mod any_against_any;
mod composite_shape_against_any;
