use crate::bounding_volume::BoundingVolume;
use crate::math::Isometry;
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor};
use crate::query::{PointQuery, Ray, RayCast, RayIntersection};
use na::RealField;
use std::any::Any;

use crate::pipeline::{BroadPhase, BroadPhaseProxyHandle, DBVTBroadPhase};

/// Bounding Volume Tree visitor collecting interferences with a given ray.
pub struct FirstRayInterferenceVisitor<'a, 'b, N: 'a + RealField, T, BV>
where
    BV: BoundingVolume<N> + RayCast<N> + PointQuery<N> + Any + Send + Sync + Clone,
    T: Any + Send + Sync,
{
    /// Ray to be tested.
    ray: &'b Ray<N>,
    handles: &'a DBVTBroadPhase<N, BV, T>,
    narrow_phase: &'a dyn Fn(T, &'b Ray<N>) -> Option<(T, RayIntersection<N>)>,
}

impl<'a, 'b, N: RealField, T, BV> FirstRayInterferenceVisitor<'a, 'b, N, T, BV>
where
    BV: BoundingVolume<N> + RayCast<N> + PointQuery<N> + Any + Send + Sync + Clone,
    T: Any + Send + Sync,
{
    /// Creates a new `FirstRayInterferenceVisitor`.
    #[inline]
    pub fn new(
        ray: &'b Ray<N>,
        handles: &'a DBVTBroadPhase<N, BV, T>,
        narrow_phase: &'a dyn Fn(T, &'b Ray<N>) -> Option<(T, RayIntersection<N>)>,
    ) -> FirstRayInterferenceVisitor<'a, 'b, N, T, BV> {
        FirstRayInterferenceVisitor {
            ray,
            handles,
            narrow_phase,
        }
    }
}

impl<'a, 'b, N, BV, T> BestFirstVisitor<N, BroadPhaseProxyHandle, BV>
    for FirstRayInterferenceVisitor<'a, 'b, N, T, BV>
where
    N: RealField,
    BV: BoundingVolume<N> + RayCast<N> + PointQuery<N> + Any + Send + Sync + Clone,
    T: Any + Send + Sync + Clone,
{
    type Result = (T, RayIntersection<N>);

    #[inline]
    fn visit(
        &mut self,
        best_cost_so_far: N,
        bv: &BV,
        value: Option<&BroadPhaseProxyHandle>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        if let Some(rough_toi) = bv.toi_with_ray(&Isometry::identity(), self.ray, true) {
            let mut res = BestFirstVisitStatus::Continue {
                cost: rough_toi,
                result: None,
            };

            if let Some(handle) = value {
                if rough_toi < best_cost_so_far {
                    if let Some((_, narrow_handle)) = self.handles.proxy(*handle) {
                        if let Some(result) = (self.narrow_phase)(narrow_handle.clone(), self.ray) {
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
            BestFirstVisitStatus::Stop
        }
    }
}
