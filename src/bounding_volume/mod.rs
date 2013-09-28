//! Bounding volumes.

// Types and traits
pub use bounding_volume::aabb::{HasAABB, AABB, WithAABB};
pub use bounding_volume::bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};

// functions
pub use bounding_volume::aabb::implicit_shape_aabb;

// modules
mod aabb;
mod bounding_volume;
