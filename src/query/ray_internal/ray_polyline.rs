use na::Real;

use query::{Ray, RayCast, RayIntersection};
use shape::{CompositeShape, Polyline};
use bounding_volume::AABB;
use partitioning::BVTCostFn;
use math::Isometry;

impl<N: Real> RayCast<N> for Polyline<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, _: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut cost_fn = PolylineRayToiCostFn {
            polyline: self,
            ray: &ls_ray,
        };

        self.bvt()
            .best_first_search(&mut cost_fn)
            .map(|(_, res)| res)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        _: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut cost_fn = PolylineRayToiAndNormalCostFn {
            polyline: self,
            ray: &ls_ray,
        };

        self.bvt()
            .best_first_search(&mut cost_fn)
            .map(|(_, mut res)| {
                res.normal = m * res.normal;
                res
            })
    }
}

/*
 * Costs functions.
 */
struct PolylineRayToiCostFn<'a, N: 'a + Real> {
    polyline: &'a Polyline<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: Real> BVTCostFn<N, usize, AABB<N>> for PolylineRayToiCostFn<'a, N> {
    type UserData = N;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        aabb.toi_with_ray(&Isometry::identity(), self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, N)> {
        self.polyline
            .segment_at(*b)
            .toi_with_ray(&Isometry::identity(), self.ray, true)
            .map(|toi| (toi, toi))
    }
}

struct PolylineRayToiAndNormalCostFn<'a, N: 'a + Real> {
    polyline: &'a Polyline<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: Real> BVTCostFn<N, usize, AABB<N>> for PolylineRayToiAndNormalCostFn<'a, N> {
    type UserData = RayIntersection<N>;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        aabb.toi_with_ray(&Isometry::identity(), self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, RayIntersection<N>)> {
        self.polyline
            .segment_at(*b)
            .toi_and_normal_with_ray(&Isometry::identity(), self.ray, true)
            .map(|inter| (inter.toi, inter))
    }
}
