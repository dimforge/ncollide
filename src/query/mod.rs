//! Non-persistant pairwise geometric queries.

pub use self::closest_points::*;
pub use self::contact::*;
pub use self::distance::*;
pub use self::point::*;
pub use self::proximity::*;
pub use self::ray::*;
pub use self::time_of_impact::*;

pub mod algorithms;
mod closest_points;
mod contact;
mod distance;
mod point;
mod proximity;
mod ray;
mod time_of_impact;
pub mod visitors;
