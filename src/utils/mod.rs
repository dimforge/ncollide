//! Various unsorted geometrical and logical operators.

pub use self::center::center;
pub use self::triangle::{circumcircle, is_point_in_triangle, triangle_area, triangle_center,
                         triangle_perimeter};
#[cfg(feature = "dim3")]
pub use self::triangle::is_affinely_dependent_triangle;
#[cfg(feature = "dim3")]
pub use self::tetrahedron::{tetrahedron_center, tetrahedron_signed_volume, tetrahedron_volume};
#[cfg(feature = "dim3")]
pub use self::cleanup::remove_unused_points;
pub use self::hashable_partial_eq::HashablePartialEq;
#[doc(inline)]
pub use self::as_bytes::AsBytes;
pub use self::median::median;
pub use self::sort::sort3;
pub use self::point_cloud_support_point::{point_cloud_support_point, point_cloud_support_point_id};
pub use self::id_allocator::{GenerationalId, IdAllocator};
pub use self::point_in_poly2d::point_in_poly2d;
pub use self::ref_with_cost::RefWithCost;
pub use self::isometry_ops::IsometryOps;
pub use self::ccw_face_normal::ccw_face_normal;
pub use self::sorted_pair::SortedPair;
pub use self::deterministic_state::DeterministicState;

mod center;
#[cfg(feature = "dim3")]
mod tetrahedron;
mod triangle;
#[cfg(feature = "dim3")]
mod cleanup;
mod hashable_partial_eq;
#[doc(hidden)]
pub mod as_bytes;
mod median;
mod sort;
mod point_cloud_support_point;
mod id_allocator;
mod point_in_poly2d;
mod ref_with_cost;
mod isometry_ops;
mod ccw_face_normal;
mod sorted_pair;
mod deterministic_state;
