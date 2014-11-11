use ray::{Ray, LocalRayCast, RayIntersection};
use math::Scalar;


/// Trait implemented by cost functions used by the best-first search on a `BVT`.
pub trait BVTCostFn<N, B, BV, R> {
    /// Computes the cost of a bounding volume.
    fn compute_bv_cost(&mut self, &BV) -> Option<N>;
    /// Computes the cost of an object, and the result to be returned if it is the best one.
    fn compute_b_cost(&mut self, &B) -> Option<(N, R)>;
}

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
