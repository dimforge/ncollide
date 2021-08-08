use crate::math::Isometry;
use crate::partitioning::{VisitStatus, Visitor};
use crate::query::{Ray, RayCast};
use na::RealField;

/// Bounding Volume Tree visitor collecting interferences with a given ray.
pub struct RayInterferencesCollector<'a, N: 'a + RealField + Copy, T: 'a> {
    /// Ray to be tested.
    pub ray: &'a Ray<N>,
    /// The maximum allowed time of impact.
    pub max_toi: N,
    /// The data contained by the nodes which bounding volume intersects `self.ray`.
    pub collector: &'a mut Vec<T>,
}

impl<'a, N: RealField + Copy, T> RayInterferencesCollector<'a, N, T> {
    /// Creates a new `RayInterferencesCollector`.
    #[inline]
    pub fn new(
        ray: &'a Ray<N>,
        max_toi: N,
        buffer: &'a mut Vec<T>,
    ) -> RayInterferencesCollector<'a, N, T> {
        RayInterferencesCollector {
            ray,
            max_toi,
            collector: buffer,
        }
    }
}

impl<'a, N, T, BV> Visitor<T, BV> for RayInterferencesCollector<'a, N, T>
where
    N: RealField + Copy,
    T: Clone,
    BV: RayCast<N>,
{
    #[inline]
    fn visit(&mut self, bv: &BV, t: Option<&T>) -> VisitStatus {
        if bv.intersects_ray(&Isometry::identity(), self.ray, self.max_toi) {
            if let Some(t) = t {
                self.collector.push(t.clone())
            }

            VisitStatus::Continue
        } else {
            VisitStatus::Stop
        }
    }
}
