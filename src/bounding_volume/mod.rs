//! Bounding volumes.

// Types and traits
pub use bounding_volume::aabb::{HasAABB, AABB, WithAABB};
pub use bounding_volume::bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};

// functions
pub use bounding_volume::aabb::implicit_shape_aabb;

// modules
mod aabb;
mod bounding_volume;

mod aabb_box;
mod aabb_cone;
mod aabb_ball;
mod aabb_cylinder;
mod aabb_capsule;
mod aabb_plane;
mod aabb_convex;
mod aabb_compound;
mod aabb_triangle;
mod aabb_segment;
