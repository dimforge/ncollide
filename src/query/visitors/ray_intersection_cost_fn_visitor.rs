use crate::bounding_volume::BoundingVolume;
use crate::math::Isometry;
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor};
use crate::query::{PointQuery, Ray, RayCast, RayIntersection};
use na::RealField;
use std::any::Any;

use crate::pipeline::{BroadPhase, BroadPhaseProxyHandle};

/// Bounding Volume Tree visitor collecting interferences with a given ray.
pub struct RayIntersectionCostFnVisitor<'a, 'b, N: 'a + RealField + Copy, T, BV>
where
    BV: BoundingVolume<N> + RayCast<N> + PointQuery<N> + Any + Send + Sync + Clone,
    T: Any + Send + Sync,
{
    /// Ray to be tested.
    ray: &'b Ray<N>,

    /// Maximum time-of-impact of the ray with the objects.
    max_toi: N,

    /// Used as a lookup to get the underlying data of the tree (uses `.query()`)
    /// This is required as the broad phase stores the data in a separate
    /// structure to the tree.
    /// TODO: Can this be made more generic?
    broad_phase: &'a dyn BroadPhase<N, BV, T>,

    /// The cost function to apply to each leaf nodes data.
    cost_fn: &'a dyn Fn(T, &'b Ray<N>, N) -> Option<(T, RayIntersection<N>)>,
}

impl<'a, 'b, N: RealField + Copy, T, BV> RayIntersectionCostFnVisitor<'a, 'b, N, T, BV>
where
    BV: BoundingVolume<N> + RayCast<N> + PointQuery<N> + Any + Send + Sync + Clone,
    T: Any + Send + Sync,
{
    /// Creates a new `RayIntersectionCostFnVisitor`.
    #[inline]
    pub fn new(
        ray: &'b Ray<N>,
        max_toi: N,
        broad_phase: &'a dyn BroadPhase<N, BV, T>,
        cost_fn: &'a dyn Fn(T, &'b Ray<N>, N) -> Option<(T, RayIntersection<N>)>,
    ) -> RayIntersectionCostFnVisitor<'a, 'b, N, T, BV> {
        RayIntersectionCostFnVisitor {
            ray,
            max_toi,
            broad_phase,
            cost_fn,
        }
    }
}

impl<'a, 'b, N, BV, T> BestFirstVisitor<N, BroadPhaseProxyHandle, BV>
    for RayIntersectionCostFnVisitor<'a, 'b, N, T, BV>
where
    N: RealField + Copy,
    BV: BoundingVolume<N> + RayCast<N> + PointQuery<N> + Any + Send + Sync + Clone,
    T: Any + Send + Sync + Clone,
{
    type Result = (T, RayIntersection<N>);

    #[inline]
    fn visit(
        &mut self,
        best_cost_so_far: N,
        bv: &BV,
        data: Option<&BroadPhaseProxyHandle>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        if let Some(rough_toi) =
            bv.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true)
        {
            let mut res = BestFirstVisitStatus::Continue {
                cost: rough_toi,
                result: None,
            };

            // If the node has data then it is a leaf
            if let Some(data_handle) = data {
                // rough_toi is less than or equal the cost of any subnode.
                // Either: The ray origin is outside the bv, and so no point in the bv
                //   could have a lower cost than rough_toi.
                // Or: The ray origin is inside the bv, and rough_toi is 0
                // We only check the data if it may be better than best_cost_so_far
                if rough_toi < best_cost_so_far {
                    // Possibly the best. Look up underlying data of the node...
                    // TODO: Should this be `.expect()`?
                    if let Some((_, leaf_data)) = self.broad_phase.proxy(*data_handle) {
                        // and then run the cost function with the nodes data
                        if let Some(result) =
                            (self.cost_fn)(leaf_data.clone(), self.ray, self.max_toi)
                        {
                            res = BestFirstVisitStatus::Continue {
                                cost: result.1.toi,
                                result: Some(result),
                            };
                        }
                    }
                };
            }

            res
        } else {
            // No intersection so we can ignore all children
            BestFirstVisitStatus::Stop
        }
    }
}
