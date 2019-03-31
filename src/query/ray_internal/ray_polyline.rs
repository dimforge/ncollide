use crate::bounding_volume::AABB;
use crate::math::Isometry;
use na::RealField;
use crate::partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor, BVH};
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::{Polyline, FeatureId};

impl<N: RealField> RayCast<N> for Polyline<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, _: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = PolylineRayToiVisitor {
            polyline: self,
            ray: &ls_ray,
        };

        self.bvt().best_first_search(&mut visitor)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        _: bool,
    ) -> Option<RayIntersection<N>>
    {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = PolylineRayToiAndNormalVisitor {
            polyline: self,
            ray: &ls_ray,
        };

        self.bvt().best_first_search(&mut visitor).map(|(best, mut res)| {
            if let FeatureId::Face(1) = res.feature {
                res.feature = FeatureId::Face(best + self.edges().len());
            } else {
                res.feature = FeatureId::Face(best);
            }

            res.normal = m * res.normal;
            res
        })
    }
}

/*
 * Costs functions.
 */
struct PolylineRayToiVisitor<'a, N: 'a + RealField> {
    polyline: &'a Polyline<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>> for PolylineRayToiVisitor<'a, N> {
    type Result = N;

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        match aabb.toi_with_ray(&Isometry::identity(), self.ray, true) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop,
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, N> {
        // FIXME: optimize this by not using Isometry identity.
        match self
            .polyline
            .segment_at(*b)
            .toi_with_ray(&Isometry::identity(), self.ray, true)
        {
            Some(toi) => BestFirstDataVisitStatus::ContinueWithResult(toi, toi),
            None => BestFirstDataVisitStatus::Continue,
        }
    }
}

struct PolylineRayToiAndNormalVisitor<'a, N: 'a + RealField> {
    polyline: &'a Polyline<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>> for PolylineRayToiAndNormalVisitor<'a, N> {
    type Result = (usize, RayIntersection<N>);

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        match aabb.toi_with_ray(&Isometry::identity(), self.ray, true) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop,
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, (usize, RayIntersection<N>)> {
        // FIXME: optimize this by not using the Isometry identity.
        match self.polyline.segment_at(*b).toi_and_normal_with_ray(
            &Isometry::identity(),
            self.ray,
            true,
        ) {
            Some(inter) => BestFirstDataVisitStatus::ContinueWithResult(inter.toi, (*b, inter)),
            None => BestFirstDataVisitStatus::Continue,
        }
    }
}
