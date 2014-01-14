//! Definition of support functions.

pub use implicit::implicit::{Implicit, HasMargin, PreferedSamplingDirections};
pub use implicit::implicit_minkowski_sum::{cso_support_point, cso_support_point_without_margin};

mod implicit;

mod implicit_box;
mod implicit_ball;
mod implicit_capsule;
mod implicit_cone;
mod implicit_cylinder;
mod implicit_convex;
mod implicit_reflection;
mod implicit_triangle;
mod implicit_segment;
mod implicit_minkowski_sum;
