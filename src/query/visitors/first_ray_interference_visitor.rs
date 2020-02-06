use crate::math::Isometry;
use crate::partitioning::{BestFirstVisitor, BestFirstVisitStatus};
use crate::query::{Ray, RayCast, RayIntersection};
use na::RealField;

use crate::pipeline::{DBVTBroadPhase, BroadPhaseProxyHandle};
use crate::pipeline::object::{CollisionObjectSet};


/// Bounding Volume Tree visitor collecting interferences with a given ray.
pub struct FirstRayInterferenceVisitor<'a, 'b, N: 'a + RealField, T, BV, Objects>
where
    Objects: CollisionObjectSet<N>,
{
    /// Ray to be tested.
    ray: &'b Ray<N>,
    solid: bool,
    handles: &'a DBVTBroadPhase<N, BV, T>,
    narrow_phase: &'a dyn Fn(T, &'b Ray<N>) -> Option<(
        Objects::CollisionObjectHandle,
        &'a Objects::CollisionObject,
        RayIntersection<N>,
    )>,
}

impl<'a, 'b, N: RealField, T, BV, Objects> FirstRayInterferenceVisitor<'a, 'b, N, T, BV, Objects>
where
    Objects: CollisionObjectSet<N>,
    BV: RayCast<N>,
{
    /// Creates a new `FirstRayInterferenceVisitor`.
    #[inline]
    pub fn new(ray: &'b Ray<N>, solid: bool, handles: &'a DBVTBroadPhase<N, BV, T>, narrow_phase: &'a dyn Fn(T, &'b Ray<N>) -> Option<(
        Objects::CollisionObjectHandle,
        &'a Objects::CollisionObject,
        RayIntersection<N>,
    )> ) ->
    FirstRayInterferenceVisitor<'a, 'b, N, T, BV, Objects> {
        FirstRayInterferenceVisitor {
            ray,
            solid,
            handles,
            narrow_phase,
        }
    }
}


impl<'a, 'b, N, BV, T, Objects> BestFirstVisitor<N, BroadPhaseProxyHandle, BV> for FirstRayInterferenceVisitor<'a, 'b, N, T, BV, Objects>
where
    N: RealField,
    BV: RayCast<N>,
    Objects: CollisionObjectSet<N>,
{
    type Result = (
        Objects::CollisionObjectHandle,
        &'a Objects::CollisionObject,
        RayIntersection<N>,
    );

    #[inline]
    fn visit(
        &mut self,
        best_cost_so_far: N,
        bv: &BV,
        value: Option<&BroadPhaseProxyHandle>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        let mut res = BestFirstVisitStatus::Stop;
        if let Some(rough_toi) = bv.toi_with_ray(&Isometry::identity(), self.ray, self.solid) {
            res = BestFirstVisitStatus::Continue {cost: rough_toi, result: None};

            if rough_toi < best_cost_so_far {
                if let Some(handle) = value {
                    if let Some((_, narrow_handle)) = self.handles.proxy(handle) {
                        if let Some(result) = (self.narrow_phase)(narrow_handle, self.ray) {
                            res = BestFirstVisitStatus::Continue {cost: result.2.toi, result: Some(result)};
                        }
                    }
                };
            }

        }

        res
    }
}
