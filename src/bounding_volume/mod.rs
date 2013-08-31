// Types and traits
pub use bounding_volume::private::aabb::{HasAABB, AABB, WithAABB};
pub use bounding_volume::private::bounding_volume::{HasBoundingVolume, BoundingVolume, LooseBoundingVolume};

// functions
pub use bounding_volume::private::aabb::implicit_shape_aabb;

// modules
mod private { // FIXME: this is only to do the compiler's work: ensure invisibility of submodules.
    #[path = "../aabb.rs"]
    mod aabb;
    #[path = "../bounding_volume.rs"]
    mod bounding_volume;
}
