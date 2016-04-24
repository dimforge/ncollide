//! Definition of support functions.

use na::{Point2, Isometry2, Point3, Isometry3};

#[doc(inline)]
pub use support_map::support_map::SupportMap;
pub use support_map::minkowski_sum_support_map::cso_support_point;
pub use support_map::utils_support_map::{point_cloud_support_point};

#[doc(hidden)]
pub mod support_map;

mod utils_support_map;
mod cuboid_support_map;
mod ball_support_map;
mod capsule_support_map;
mod cone_support_map;
mod cylinder_support_map;
mod convex_support_map;
mod reflection_support_map;
mod triangle_support_map;
mod segment_support_map;
mod minkowski_sum_support_map;
// mod implicit_spec;

/// A 2-dimensional support-map trait object.
pub type SupportMap2<N> = SupportMap<Point2<N>, Isometry2<N>>;
/// A 3-dimensional support-map trait object.
pub type SupportMap3<N> = SupportMap<Point3<N>, Isometry3<N>>;
