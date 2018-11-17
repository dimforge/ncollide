//! Various unsorted geometrical and logical operators.

#[doc(inline)]
pub use self::as_bytes::AsBytes;
pub use self::ccw_face_normal::ccw_face_normal;
pub use self::center::center;
#[cfg(feature = "dim3")]
pub use self::cleanup::remove_unused_points;
pub use self::deterministic_state::DeterministicState;
pub use self::hashable_partial_eq::HashablePartialEq;
pub use self::id_allocator::{GenerationalId, IdAllocator};
pub use self::isometry_ops::IsometryOps;
pub use self::median::median;
pub use self::point_cloud_support_point::{
    point_cloud_support_point, point_cloud_support_point_id,
};
pub use self::point_in_poly2d::point_in_poly2d;
pub use self::ref_with_cost::RefWithCost;
pub use self::sort::sort3;
pub use self::sorted_pair::SortedPair;
#[cfg(feature = "dim3")]
pub use self::tetrahedron::{tetrahedron_center, tetrahedron_signed_volume, tetrahedron_volume};
#[cfg(feature = "dim3")]
pub use self::triangle::is_affinely_dependent_triangle;
pub use self::triangle::{
    circumcircle, is_point_in_triangle, triangle_area, triangle_center, triangle_perimeter,
};

#[doc(hidden)]
pub mod as_bytes;
mod ccw_face_normal;
mod center;
#[cfg(feature = "dim3")]
mod cleanup;
mod deterministic_state;
mod hashable_partial_eq;
mod id_allocator;
mod isometry_ops;
mod median;
mod point_cloud_support_point;
mod point_in_poly2d;
mod ref_with_cost;
mod sort;
mod sorted_pair;
#[cfg(feature = "dim3")]
mod tetrahedron;
mod triangle;
