//! Spatial partitioning tools.

pub use partitioning::dbvt::{DBVT, DBVTLeaf};
pub use partitioning::bvt::{BVT, median_partitioner, median_partitioner_with_centers,
                            BinaryPartition, BVTNode, Internal, Leaf};
#[doc(inline)]
pub use partitioning::bvt_visitor::{BVTVisitor,
                                    RayInterferencesCollector,
                                    BoundingVolumeInterferencesCollector,
                                    PointInterferencesCollector};
#[doc(inline)]
pub use partitioning::bvtt_visitor::BVTTVisitor;
#[doc(inline)]
pub use partitioning::bvt_cost_fn::{BVTCostFn, RayIntersectionCostFn};

mod dbvt;
mod bvt;

#[doc(hidden)]
pub mod bvt_visitor;
#[doc(hidden)]
pub mod bvtt_visitor;
#[doc(hidden)]
pub mod bvt_cost_fn;
