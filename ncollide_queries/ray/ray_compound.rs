use na::Translate;
use entities::bounding_volume::AABB;
use entities::shape::Compound;
use entities::partitioning::BVTCostFn;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use math::{Scalar, Point, Vect, Isometry};


// XXX: if solid == false, this might return internal intersection.
impl<N, P, V, M> LocalRayCast<N, P, V> for Compound<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    fn toi_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        let mut cost_fn = CompoundRayToiCostFn { compound: self, ray: ray, solid: solid };

        self.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res)
    }

    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        let mut cost_fn = CompoundRayToiAndNormalCostFn { compound: self, ray: ray, solid: solid };

        self.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res)
    }

    // XXX: We have to implement toi_and_normal_and_uv_with_ray! Otherwise, no uv will be computed
    // for any of the sub-shapes.
}

impl<N, P, V, M> RayCast<N, P, V, M> for Compound<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
}

/*
 * Costs functions.
 */
struct CompoundRayToiCostFn<'a, N: 'a, P: 'a, V: 'a, M: 'a> {
    compound: &'a Compound<N, P, V, M>,
    ray:      &'a Ray<P, V>,
    solid:    bool
}

impl<'a, N, P, V, M> BVTCostFn<N, usize, AABB<P>, N> for CompoundRayToiCostFn<'a, N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        aabb.toi_with_ray(self.ray, self.solid)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, N)> {
        let elt = &self.compound.shapes()[*b];
        elt.1.toi_with_transform_and_ray(&elt.0, self.ray, self.solid).map(|toi| (toi, toi))
    }
}

struct CompoundRayToiAndNormalCostFn<'a, N: 'a, P: 'a, V: 'a, M: 'a> {
    compound: &'a Compound<N, P, V, M>,
    ray:      &'a Ray<P, V>,
    solid:    bool
}

impl<'a, N, P, V, M> BVTCostFn<N, usize, AABB<P>, RayIntersection<N, V>>
for CompoundRayToiAndNormalCostFn<'a, N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        aabb.toi_with_ray(self.ray, self.solid)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, RayIntersection<N, V>)> {
        let elt = &self.compound.shapes()[*b];
        elt.1.toi_and_normal_with_transform_and_ray(&elt.0, self.ray, self.solid).map(|inter| (inter.toi, inter))
    }
}
