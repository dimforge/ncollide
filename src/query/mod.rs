//! Non-persistent geometric queries.
//!
//! # General cases
//! The most general methods provided by this module are:
//!
//! * [`closest_points`](query::closest_points()) to compute the closest points between two shapes.
//! * [`distance`](query::distance()) to compute the distance between two shapes.
//! * [`contact`](query::contact()) to compute one pair of contact points between two shapes, including penetrating contact.
//! * [`proximity`](query::proximity()) to determine if two shapes are intersecting or not.
//! * [`time_of_impact`](query::time_of_impact()) to determine when two shapes undergoing translational motions hit for the first time.
//!
//! Ray-casting and point-projection can be achieved by importing traits:
//!
//! * [`RayCast`](query::RayCast) for ray-casting.
//! * [`PointQuery`](query::PointQuery) for point projection.
//!
//! # Specific cases
//! All the other functions exported by this module are more specific versions of the ones described above.
//! For example `distance_ball_ball` computes the distance between two shapes known at compile-time to be balls.
//! They are less convenient to use than the most generic version but will be slightly faster due to the lack of dynamic dispatch.
//! Generally, the specific functions have the form `[operation]_[shape1]_[shape2]()` where:
//!
//! * `[operation]` can be `closest_points`, `distance`, `contact`, `proximity` or `time_of_impact`.
//! * `[shape1]` is the type of the first shape passed to the function, e.g., `ball`, or `plane`.
//! * `[shape2]` is the type of the second shape passed to the function, e.g., `ball`, or `plane`.

pub use self::closest_points::*;
pub use self::contact::*;
pub use self::distance::*;
pub use self::point::*;
pub use self::proximity::*;
pub use self::ray::*;
pub use self::time_of_impact::*;
pub use self::nonlinear_time_of_impact::*;

pub mod algorithms;
mod closest_points;
mod contact;
mod distance;
mod point;
mod proximity;
mod ray;
mod time_of_impact;
mod nonlinear_time_of_impact;
pub mod visitors;
