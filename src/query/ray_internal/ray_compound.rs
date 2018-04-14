use alga::general::Id;
use bounding_volume::AABB;
use shape::Compound;
use partitioning::BVTCostFn;
use query::{Ray, RayCast, RayIntersection};
use math::{Isometry, Point};

// XXX: if solid == false, this might return internal intersection.
impl<N: Real> RayCast<P, M> for Compound<N> {
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<P>, solid: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut cost_fn = CompoundRayToiCostFn {
            compound: self,
            ray: &ls_ray,
            solid: solid,
        };

        self.bvt()
            .best_first_search(&mut cost_fn)
            .map(|(_, res)| res)
    }

    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<Vector<N>>> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut cost_fn = CompoundRayToiAndNormalCostFn {
            compound: self,
            ray: &ls_ray,
            solid: solid,
        };

        self.bvt()
            .best_first_search(&mut cost_fn)
            .map(|(_, mut res)| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
    }

    // XXX: We have to implement toi_and_normal_and_uv_with_ray! Otherwise, no uv will be computed
    // for any of the sub-shapes.
}

/*
 * Costs functions.
 */
struct CompoundRayToiCostFn<'a, P: 'a + Point, M: 'a + Isometry<P>> {
    compound: &'a Compound<N>,
    ray: &'a Ray<P>,
    solid: bool,
}

impl<'a, P, M> BVTCostFn<N, usize, AABB<N>> for CompoundRayToiCostFn<'a, P, M>
where
    N: Real,
    M: Isometry<P>,
{
    type UserData = N;
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        aabb.toi_with_ray(&Id::new(), self.ray, self.solid)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, N)> {
        let elt = &self.compound.shapes()[*b];
        elt.1
            .toi_with_ray(&elt.0, self.ray, self.solid)
            .map(|toi| (toi, toi))
    }
}

struct CompoundRayToiAndNormalCostFn<'a, P: 'a + Point, M: 'a + Isometry<P>> {
    compound: &'a Compound<N>,
    ray: &'a Ray<P>,
    solid: bool,
}

impl<'a, N: Real> BVTCostFn<N, usize, AABB<N>>
    for CompoundRayToiAndNormalCostFn<'a, P, M> {
    type UserData = RayIntersection<Vector<N>>;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        aabb.toi_with_ray(&Id::new(), self.ray, self.solid)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, RayIntersection<Vector<N>>)> {
        let elt = &self.compound.shapes()[*b];
        elt.1
            .toi_and_normal_with_ray(&elt.0, self.ray, self.solid)
            .map(|inter| (inter.toi, inter))
    }
}
