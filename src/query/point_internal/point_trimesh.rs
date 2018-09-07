use bounding_volume::AABB;
use math::{Isometry, Point};
use na::{self, Real};
use partitioning::BVTCostFn;
use query::{PointProjection, PointQuery, PointQueryWithLocation, visitors::PointContainmentTest};
use shape::{FeatureId, TrianglePointLocation, TriMesh};
use utils::IsometryOps;

impl<N: Real> PointQuery<N> for TriMesh<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        let (projection, _) = self.project_point_with_location(m, point, solid);
        projection
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        _: &Isometry<N>,
        _: &Point<N>,
    ) -> (PointProjection<N>, FeatureId) {
        unimplemented!()
    }

    // FIXME: implement distance_to_point too?

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, point: &Point<N>) -> bool {
        let ls_pt = m.inverse_transform_point(point);
        let mut visitor = PointContainmentTest {
            shape: self,
            point: &ls_pt,
            found: false,
        };

        self.bvt().visit(&mut visitor);

        visitor.found
    }
}

impl<N: Real> PointQueryWithLocation<N> for TriMesh<N> {
    type Location = (usize, TrianglePointLocation<N>);

    #[inline]
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
        _: bool,
    ) -> (PointProjection<N>, Self::Location) {
        let ls_pt = m.inverse_transform_point(point);
        let mut cost_fn = TriMeshPointProjCostFn {
            polyline: self,
            point: &ls_pt,
        };

        let (mut proj, extra_info) = self.bvt().best_first_search(&mut cost_fn).unwrap().1;
        proj.point = m * proj.point;

        (proj, extra_info)
    }
}

/*
 * Costs function.
 */
struct TriMeshPointProjCostFn<'a, N: 'a + Real> {
    polyline: &'a TriMesh<N>,
    point: &'a Point<N>,
}

impl<'a, N: Real> BVTCostFn<N, usize, AABB<N>> for TriMeshPointProjCostFn<'a, N> {
    type UserData = (PointProjection<N>, (usize, TrianglePointLocation<N>));

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        Some(aabb.distance_to_point(&Isometry::identity(), self.point, true))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, Self::UserData)> {
        let (proj, extra_info) = self.polyline.triangle_at(*b).project_point_with_location(
            &Isometry::identity(),
            self.point,
            true,
        );

        let extra_info = (*b, extra_info);
        Some((na::distance(self.point, &proj.point), (proj, extra_info)))
    }
}
