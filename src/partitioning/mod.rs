//! Spatial partitioning tools.

pub use self::bvt::{BinaryPartition, BVT, BVTNode};
#[doc(inline)]
pub use self::bvt_cost_fn::BVTCostFn;
pub use self::dbvt::{DBVT, DBVTLeaf, DBVTLeafId};
pub use self::partitioning_structure::PartitioningStructure;
pub use self::simultaneous_visitor::{simultaneous_visit, SimultaneousVisitor};
pub use self::visitor::{visit, Visitor, VisitStatus};

mod dbvt;
mod bvt;

#[doc(hidden)]
pub mod bvt_cost_fn;
mod partitioning_structure;
mod visitor;
mod simultaneous_visitor;