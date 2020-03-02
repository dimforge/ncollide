use crate::bounding_volume::AABB;
use crate::math::Isometry;
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor, BVH};
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::{FeatureId, Polyline};
use na::RealField;

impl<N: RealField> RayCast<N> for Polyline<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, max_toi: N, _: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = PolylineRayToiVisitor {
            polyline: self,
            ray: &ls_ray,
            max_toi,
        };

        self.bvt().best_first_search(&mut visitor).map(|res| res.1)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        _: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = PolylineRayToiAndNormalVisitor {
            polyline: self,
            ray: &ls_ray,
            max_toi,
        };

        self.bvt()
            .best_first_search(&mut visitor)
            .map(|(_, (best, mut res))| {
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
    max_toi: N,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>> for PolylineRayToiVisitor<'a, N> {
    type Result = N;

    #[inline]
    fn visit(
        &mut self,
        best: N,
        aabb: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        if let Some(toi) = aabb.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true) {
            let mut res = BestFirstVisitStatus::Continue {
                cost: toi,
                result: None,
            };

            if let Some(b) = data {
                if toi < best {
                    // FIXME: optimize this by not using Isometry identity.
                    let segment = self.polyline.segment_at(*b);
                    if let Some(toi) =
                        segment.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true)
                    {
                        res = BestFirstVisitStatus::Continue {
                            cost: toi,
                            result: Some(toi),
                        }
                    }
                }
            }

            res
        } else {
            BestFirstVisitStatus::Stop
        }
    }
}

struct PolylineRayToiAndNormalVisitor<'a, N: 'a + RealField> {
    polyline: &'a Polyline<N>,
    ray: &'a Ray<N>,
    max_toi: N,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>>
    for PolylineRayToiAndNormalVisitor<'a, N>
{
    type Result = (usize, RayIntersection<N>);

    #[inline]
    fn visit(
        &mut self,
        best: N,
        aabb: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        if let Some(toi) = aabb.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true) {
            let mut res = BestFirstVisitStatus::Continue {
                cost: toi,
                result: None,
            };

            if let Some(b) = data {
                if toi < best {
                    // FIXME: optimize this by not using Isometry identity.
                    let segment = self.polyline.segment_at(*b);
                    if let Some(toi) = segment.toi_and_normal_with_ray(
                        &Isometry::identity(),
                        self.ray,
                        self.max_toi,
                        true,
                    ) {
                        res = BestFirstVisitStatus::Continue {
                            cost: toi.toi,
                            result: Some((*b, toi)),
                        };
                    }
                }
            }

            res
        } else {
            BestFirstVisitStatus::Stop
        }
    }
}
