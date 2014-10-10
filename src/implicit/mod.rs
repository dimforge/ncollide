//! Definition of support functions.

#[doc(inline)]
pub use implicit::implicit::{Implicit, PreferedSamplingDirections};
pub use implicit::implicit_minkowski_sum::cso_support_point;
pub use implicit::implicit_utils::{point_cloud_support_point};

#[doc(hidden)]
pub mod implicit;

mod implicit_utils;
mod implicit_cuboid;
mod implicit_ball;
mod implicit_capsule;
mod implicit_cone;
mod implicit_cylinder;
mod implicit_convex;
mod implicit_reflection;
mod implicit_triangle;
mod implicit_segment;
mod implicit_minkowski_sum;
