use alga::general::Id;
use bounding_volume::AABB;
use shape::Compound;
use partitioning::BVTCostFn;
use query::{Ray, RayCast, RayIntersection};
use math::{Isometry, Point};

// XXX: if solid == false, this might return internal intersection.
impl<P: Point, M: Isometry<P>> RayCast<P, M> for Compound<P, M> {
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<P::Real> {
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
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
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
struct CompoundRayToiCostFn<'a, P: 'a + Point, M: 'a> {
    compound: &'a Compound<P, M>,
    ray: &'a Ray<P>,
    solid: bool,
}

impl<'a, P, M> BVTCostFn<P::Real, usize, AABB<P>> for CompoundRayToiCostFn<'a, P, M>
where
    P: Point,
    M: Isometry<P>,
{
    type UserData = P::Real;
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<P::Real> {
        aabb.toi_with_ray(&Id::new(), self.ray, self.solid)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(P::Real, P::Real)> {
        let elt = &self.compound.shapes()[*b];
        elt.1
            .toi_with_ray(&elt.0, self.ray, self.solid)
            .map(|toi| (toi, toi))
    }
}

struct CompoundRayToiAndNormalCostFn<'a, P: 'a + Point, M: 'a> {
    compound: &'a Compound<P, M>,
    ray: &'a Ray<P>,
    solid: bool,
}

impl<'a, P: Point, M: Isometry<P>> BVTCostFn<P::Real, usize, AABB<P>>
    for CompoundRayToiAndNormalCostFn<'a, P, M> {
    type UserData = RayIntersection<P::Vector>;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<P::Real> {
        aabb.toi_with_ray(&Id::new(), self.ray, self.solid)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(P::Real, RayIntersection<P::Vector>)> {
        let elt = &self.compound.shapes()[*b];
        elt.1
            .toi_and_normal_with_ray(&elt.0, self.ray, self.solid)
            .map(|inter| (inter.toi, inter))
    }
}
