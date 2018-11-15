use bounding_volume::AABB;
use math::Isometry;
use na::Real;
use partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor, BVH};
use query::{Ray, RayCast, RayIntersection};
use shape::Compound;

// XXX: if solid == false, this might return internal intersection.
impl<N: Real> RayCast<N> for Compound<N> {
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, solid: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = CompoundRayToiVisitor {
            compound: self,
            ray: &ls_ray,
            solid: solid,
        };

        self.bvt().best_first_search(&mut visitor)
    }

    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = CompoundRayToiAndNormalVisitor {
            compound: self,
            ray: &ls_ray,
            solid: solid,
        };

        self.bvt().best_first_search(&mut visitor).map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }

    // XXX: We have to implement toi_and_normal_and_uv_with_ray! Otherwise, no uv will be computed
    // for any of the sub-shapes.
}

/*
 * Costs functions.
 */
struct CompoundRayToiVisitor<'a, N: 'a + Real> {
    compound: &'a Compound<N>,
    ray: &'a Ray<N>,
    solid: bool,
}

impl<'a, N: Real> BestFirstVisitor<N, usize, AABB<N>> for CompoundRayToiVisitor<'a, N> {
    type Result = N;

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        match aabb.toi_with_ray(&Isometry::identity(), self.ray, self.solid) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop,
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, N> {
        let elt = &self.compound.shapes()[*b];
        match elt.1.toi_with_ray(&elt.0, self.ray, self.solid) {
            Some(toi) => BestFirstDataVisitStatus::ContinueWithResult(toi, toi),
            None => BestFirstDataVisitStatus::Continue,
        }
    }
}

struct CompoundRayToiAndNormalVisitor<'a, N: 'a + Real> {
    compound: &'a Compound<N>,
    ray: &'a Ray<N>,
    solid: bool,
}

impl<'a, N: Real> BestFirstVisitor<N, usize, AABB<N>> for CompoundRayToiAndNormalVisitor<'a, N> {
    type Result = RayIntersection<N>;

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        match aabb.toi_with_ray(&Isometry::identity(), self.ray, self.solid) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop,
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, RayIntersection<N>> {
        let elt = &self.compound.shapes()[*b];
        match elt.1.toi_and_normal_with_ray(&elt.0, self.ray, self.solid) {
            Some(inter) => BestFirstDataVisitStatus::ContinueWithResult(inter.toi, inter),
            None => BestFirstDataVisitStatus::Continue,
        }
    }
}
