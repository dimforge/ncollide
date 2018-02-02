//! Spatial partitioning tools.

pub use partitioning::dbvt2::{DBVT2, DBVTLeaf2, DBVTLeafId};
pub use partitioning::dbvt::{DBVTLeaf, DBVT};
pub use partitioning::bvt::{BVTNode, BinaryPartition, BVT};
#[doc(inline)]
pub use partitioning::bvt_visitor::{BVTVisitor, BoundingVolumeInterferencesCollector};
#[doc(inline)]
pub use partitioning::bvtt_visitor::BVTTVisitor;
#[doc(inline)]
pub use partitioning::bvt_cost_fn::BVTCostFn;

mod dbvt;
mod dbvt2;
mod bvt;

#[doc(hidden)]
pub mod bvt_visitor;
#[doc(hidden)]
pub mod bvtt_visitor;
#[doc(hidden)]
pub mod bvt_cost_fn;
