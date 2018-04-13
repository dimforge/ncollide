// pub use center::center;
// // pub use project_homogeneous::{project_homogeneous, project_homogeneous_to};
// pub use triangle::{circumcircle, is_point_in_triangle, triangle_area, triangle_center,
//                    triangle_perimeter, is_affinely_dependent_triangle3};
// pub use tetrahedron::{tetrahedron_center, tetrahedron_signed_volume, tetrahedron_volume};
// pub use cleanup::remove_unused_points;
// pub use derivatives::{binom, dcos, dsin};
// // pub use optimization::{maximize_with_newton, newton, minimize_with_bfgs, bfgs,
// //                        LineSearch, BacktrackingLineSearch};
// pub use hashable_partial_eq::HashablePartialEq;
// #[doc(inline)]
// pub use as_bytes::AsBytes;
// // pub use cov::{cov, cov_and_center, center_reduce};
pub use self::median::median;
// pub use sort::sort3;
// pub use cross3::cross3;
// pub use perp2::perp2;
// pub use point_cloud_support_point::{point_cloud_support_point, point_cloud_support_point_id};
// pub use repeat::repeat;
pub use self::id_allocator::{GenerationalId, IdAllocator};
// pub use point_in_poly2d::point_in_poly2d;
pub use self::ref_with_cost::RefWithCost;
pub use self::isometry_ops::IsometryOps;
pub use self::ccw_face_normal::ccw_face_normal;

// pub mod data;
// mod center;
// // mod project_homogeneous;
// mod tetrahedron;
// mod triangle;
// mod cleanup;
// mod derivatives;
// // mod optimization;
// mod hashable_partial_eq;
// #[doc(hidden)]
// pub mod as_bytes;
// // mod cov;
mod median;
// mod sort;
// mod cross3;
// mod perp2;
// mod point_cloud_support_point;
// mod repeat;
mod id_allocator;
// mod point_in_poly2d;
mod ref_with_cost;
mod isometry_ops;
mod ccw_face_normal;