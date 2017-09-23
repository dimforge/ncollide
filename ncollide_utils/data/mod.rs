//! Data structure utilities.

pub use self::sparse_vec::SparseVec;

pub mod pair;
pub mod hash;
pub mod hash_map;
pub mod owned_allocation_cache;
pub mod vec_slice;
pub mod ref_with_cost;
pub mod uid_remap;
pub mod vec_map;
mod sparse_vec;
