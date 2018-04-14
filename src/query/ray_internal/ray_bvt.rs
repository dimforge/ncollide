use na::Real;

use math::Isometry;
use partitioning::{BVTCostFn, BVTVisitor};
use query::{Ray, RayCast, RayIntersection};

/// A search thet selects the objects that has the smallest time of impact with a given ray.
pub struct RayIntersectionCostFn<'a, N: 'a + Real> {
    ray: &'a Ray<N>,
    solid: bool,
    uvs: bool,
}

impl<'a, N: Real> RayIntersectionCostFn<'a, N> {
    /// Creates a new `BestRayInterferenceSearch`.
    pub fn new(ray: &'a Ray<N>, solid: bool, uvs: bool) -> RayIntersectionCostFn<'a, N> {
        RayIntersectionCostFn {
            ray: ray,
            solid: solid,
            uvs: uvs,
        }
    }
}

impl<'a, N, B, BV> BVTCostFn<N, B, BV> for RayIntersectionCostFn<'a, N>
where
    N: Real,
    B: RayCast<N>,
    BV: RayCast<N>,
{
    type UserData = RayIntersection<N>;

    #[inline]
    fn compute_bv_cost(&mut self, bv: &BV) -> Option<N> {
        bv.toi_with_ray(&Isometry::identity(), self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &B) -> Option<(N, RayIntersection<N>)> {
        if self.uvs {
            b.toi_and_normal_and_uv_with_ray(&Isometry::identity(), self.ray, self.solid)
                .map(|i| (i.toi, i))
        } else {
            b.toi_and_normal_with_ray(&Isometry::identity(), self.ray, self.solid)
                .map(|i| (i.toi, i))
        }
    }
}

/// Bounding Volume Tree visitor collecting interferences with a given ray.
pub struct RayInterferencesCollector<'a, N: 'a + Real, B: 'a> {
    ray: &'a Ray<N>,
    collector: &'a mut Vec<B>,
}

impl<'a, N: Real, B> RayInterferencesCollector<'a, N, B> {
    /// Creates a new `RayInterferencesCollector`.
    #[inline]
    pub fn new(ray: &'a Ray<N>, buffer: &'a mut Vec<B>) -> RayInterferencesCollector<'a, N, B> {
        RayInterferencesCollector {
            ray: ray,
            collector: buffer,
        }
    }
}

impl<'a, N, B, BV> BVTVisitor<B, BV> for RayInterferencesCollector<'a, N, B>
where
    N: Real,
    B: Clone,
    BV: RayCast<N>,
{
    #[inline]
    fn visit_internal(&mut self, bv: &BV) -> bool {
        bv.intersects_ray(&Isometry::identity(), self.ray)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &B, bv: &BV) {
        if bv.intersects_ray(&Isometry::identity(), self.ray) {
            self.collector.push(b.clone())
        }
    }
}
