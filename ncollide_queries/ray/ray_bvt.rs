use na::Identity;
use math::{Point, Vect};
use entities::partitioning::{BVTCostFn, BVTVisitor};
use ray::{RayCast, Ray, RayIntersection};

/// A search thet selects the objects that has the smallest time of impact with a given ray.
pub struct RayIntersectionCostFn<'a, P: 'a + Point> {
    ray:   &'a Ray<P>,
    solid: bool,
    uvs:   bool
}

impl<'a, P: Point> RayIntersectionCostFn<'a, P> {
    /// Creates a new `BestRayInterferenceSearch`.
    pub fn new(ray: &'a Ray<P>, solid: bool, uvs: bool) -> RayIntersectionCostFn<'a, P> {
        RayIntersectionCostFn {
            ray:   ray,
            solid: solid,
            uvs:   uvs
        }
    }
}

impl<'a, P, B, BV> BVTCostFn<<P::Vect as Vect>::Scalar, B, BV, RayIntersection<P::Vect>>
for RayIntersectionCostFn<'a, P>
    where P:  Point,
          B:  RayCast<P, Identity>,
          BV: RayCast<P, Identity> {
    #[inline]
    fn compute_bv_cost(&mut self, bv: &BV) -> Option<<P::Vect as Vect>::Scalar> {
        bv.toi_with_ray(&Identity::new(), self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &B) -> Option<(<P::Vect as Vect>::Scalar, RayIntersection<P::Vect>)> {
        if self.uvs {
            b.toi_and_normal_and_uv_with_ray(&Identity::new(), self.ray, self.solid).map(|i| (i.toi, i))
        }
        else {
            b.toi_and_normal_with_ray(&Identity::new(), self.ray, self.solid).map(|i| (i.toi, i))
        }
    }
}

/// Bounding Volume Tree visitor collecting interferences with a given ray.
pub struct RayInterferencesCollector<'a, P: 'a + Point, B: 'a> {
    ray:       &'a Ray<P>,
    collector: &'a mut Vec<B>
}

impl<'a, P: Point, B> RayInterferencesCollector<'a, P, B> {
    /// Creates a new `RayInterferencesCollector`.
    #[inline]
    pub fn new(ray: &'a Ray<P>, buffer: &'a mut Vec<B>) -> RayInterferencesCollector<'a, P, B> {
        RayInterferencesCollector {
            ray:       ray,
            collector: buffer
        }
    }
}

impl<'a, P, B, BV> BVTVisitor<B, BV> for RayInterferencesCollector<'a, P, B>
    where P:  Point,
          B:  Clone,
          BV: RayCast<P, Identity> {
    #[inline]
    fn visit_internal(&mut self, bv: &BV) -> bool {
        bv.intersects_ray(&Identity::new(), self.ray)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &B, bv: &BV) {
        if bv.intersects_ray(&Identity::new(), self.ray) {
            self.collector.push(b.clone())
        }
    }
}
