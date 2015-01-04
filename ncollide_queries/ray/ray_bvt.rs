use math::Scalar;
use entities::partitioning::{BVTCostFn, BVTVisitor};
use ray::{Ray, LocalRayCast, RayIntersection};

/// A search thet selects the objects that has the smallest time of impact with a given ray.
pub struct RayIntersectionCostFn<'a, N, P: 'a, V: 'a> {
    ray:   &'a Ray<P, V>,
    solid: bool,
    uvs:   bool
}

impl<'a, N, P, V> RayIntersectionCostFn<'a, N, P, V> {
    /// Creates a new `BestRayInterferenceSearch`.
    pub fn new(ray: &'a Ray<P, V>, solid: bool, uvs: bool) -> RayIntersectionCostFn<'a, N, P, V> {
        RayIntersectionCostFn {
            ray:   ray,
            solid: solid,
            uvs:   uvs
        }
    }
}

impl<'a, N, P, V, B, BV> BVTCostFn<N, B, BV, RayIntersection<N, V>> for RayIntersectionCostFn<'a, N, P, V>
    where N:  Scalar,
          B:  LocalRayCast<N, P, V>,
          BV: LocalRayCast<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, bv: &BV) -> Option<N> {
        bv.toi_with_ray(self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &B) -> Option<(N, RayIntersection<N, V>)> {
        if self.uvs {
            b.toi_and_normal_and_uv_with_ray(self.ray, self.solid).map(|i| (i.toi, i))
        }
        else {
            b.toi_and_normal_with_ray(self.ray, self.solid).map(|i| (i.toi, i))
        }
    }
}

/// Bounding Volume Tree visitor collecting interferences with a given ray.
pub struct RayInterferencesCollector<'a, P: 'a, V: 'a, B: 'a> {
    ray:       &'a Ray<P, V>,
    collector: &'a mut Vec<B>
}

impl<'a, P, V, B> RayInterferencesCollector<'a, P, V, B> {
    /// Creates a new `RayInterferencesCollector`.
    #[inline]
    pub fn new(ray: &'a Ray<P, V>, buffer: &'a mut Vec<B>) -> RayInterferencesCollector<'a, P, V, B> {
        RayInterferencesCollector {
            ray:       ray,
            collector: buffer
        }
    }
}

impl<'a, N, P, V, B: Clone, BV: LocalRayCast<N, P, V>> BVTVisitor<B, BV> for RayInterferencesCollector<'a, P, V, B> {
    #[inline]
    fn visit_internal(&mut self, bv: &BV) -> bool {
        bv.intersects_ray(self.ray)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &B, bv: &BV) {
        if bv.intersects_ray(self.ray) {
            self.collector.push(b.clone())
        }
    }
}
