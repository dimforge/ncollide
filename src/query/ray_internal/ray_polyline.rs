use bounding_volume::AABB;
use math::Isometry;
use na::Real;
use partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor, BVH};
use query::{Ray, RayCast, RayIntersection};
use shape::Polyline;

impl<N: Real> RayCast<N> for Polyline<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, _: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = PolylineRayToiVisitor {
            polyline: self,
            ray: &ls_ray,
        };

        self.bvt()
            .best_first_search(&mut visitor)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        _: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = PolylineRayToiAndNormalVisitor {
            polyline: self,
            ray: &ls_ray,
        };

        self.bvt()
            .best_first_search(&mut visitor)
            .map(|mut res| {
                res.normal = m * res.normal;
                res
            })
    }
}

/*
 * Costs functions.
 */
struct PolylineRayToiVisitor<'a, N: 'a + Real> {
    polyline: &'a Polyline<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: Real> BestFirstVisitor<N, usize, AABB<N>> for PolylineRayToiVisitor<'a, N> {
    type Result = N;

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        match aabb.toi_with_ray(&Isometry::identity(), self.ray, true) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, N> {
        // FIXME: optimize this by not using Isometry identity.
        match self.polyline
            .segment_at(*b)
            .toi_with_ray(&Isometry::identity(), self.ray, true) {
            Some(toi) => BestFirstDataVisitStatus::ContinueWithResult(toi, toi),
            None => BestFirstDataVisitStatus::Continue
        }
    }
}

struct PolylineRayToiAndNormalVisitor<'a, N: 'a + Real> {
    polyline: &'a Polyline<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: Real> BestFirstVisitor<N, usize, AABB<N>> for PolylineRayToiAndNormalVisitor<'a, N> {
    type Result = RayIntersection<N>;

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        match aabb.toi_with_ray(&Isometry::identity(), self.ray, true) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, RayIntersection<N>> {
        // FIXME: optimize this by not using the Isometry identity.
        match self.polyline
            .segment_at(*b)
            .toi_and_normal_with_ray(&Isometry::identity(), self.ray, true) {
            Some(inter) => BestFirstDataVisitStatus::ContinueWithResult(inter.toi, inter),
            None => BestFirstDataVisitStatus::Continue
        }
    }
}
