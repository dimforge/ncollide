#[link(name     = "ncollide"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "833dfdbd-3e85-456f-9ca7-84aade5ed3be")];
#[crate_type = "lib"];
#[deny(non_camel_case_types)];
#[deny(non_uppercase_statics)];
#[deny(unnecessary_qualification)];
#[warn(missing_doc)];

extern mod std;
extern mod extra;
extern mod nalgebra;

/// Bounding volumes.
pub mod bounding_volume;

/// Geometries.
pub mod geom;

/// Ray casting utilities.
pub mod ray;

/// Narrow phases.
pub mod narrow;

/// Broad phases.
pub mod broad;

/// Main data structure for contacts.
pub mod contact;

/// Spatial partitioning tools.
pub mod partitioning {
    /// A Dynamic Bounding Volume Tree.
    pub mod dbvt;
    /// A read-only Bounding Volume Tree.
    pub mod bvt;
    /// Trait of visitors of bounding volume based tree.
    pub mod bvt_visitor;
}

/// Data structure utilities.
pub mod util {
    pub mod pair;
    pub mod hash;
    pub mod hash_map;
    pub mod owned_allocation_cache;
}

#[cfg(test)]
mod tests {
    mod geom;
    mod narrow;
    mod algo;
}
