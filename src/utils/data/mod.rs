//! Data structure utilities.

#[doc(inline)]
pub use self::has_uid::HasUid;

pub mod pair;
pub mod hash;
pub mod hash_map;
pub mod owned_allocation_cache;
pub mod vec_slice;
pub mod ref_with_cost;
pub mod has_uid_map;

#[doc(hidden)]
pub mod has_uid;
