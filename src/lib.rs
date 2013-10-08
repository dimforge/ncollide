#[link(name     = "ncollide"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "833dfdbd-3e85-456f-9ca7-84aade5ed3be")];
#[crate_type = "lib"];
#[deny(non_camel_case_types)];
#[deny(non_uppercase_statics)];
#[deny(unnecessary_qualification)];
#[warn(missing_doc)];
#[feature(macro_rules)];

extern mod std;
extern mod extra;
extern mod nalgebra;

pub mod bounding_volume;
pub mod geom;
pub mod ray;
pub mod narrow;
pub mod broad;
pub mod contact;

/// Spatial partitioning tools.
pub mod partitioning {
    pub mod dbvt;
    pub mod bvt;
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
