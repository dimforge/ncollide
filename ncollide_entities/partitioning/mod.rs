//! Spatial partitioning tools.

pub use partitioning::dbvt::{DBVT, DBVTLeaf};
pub use partitioning::bvt::{BVT, median_partitioner, median_partitioner_with_centers,
                            BinaryPartition, BVTNode};
#[doc(inline)]
pub use partitioning::bvt_visitor::{BVTVisitor, BoundingVolumeInterferencesCollector};
#[doc(inline)]
pub use partitioning::bvtt_visitor::BVTTVisitor;
#[doc(inline)]
pub use partitioning::bvt_cost_fn::BVTCostFn;

mod dbvt;
mod bvt;

#[doc(hidden)]
pub mod bvt_visitor;
#[doc(hidden)]
pub mod bvtt_visitor;
#[doc(hidden)]
pub mod bvt_cost_fn;
