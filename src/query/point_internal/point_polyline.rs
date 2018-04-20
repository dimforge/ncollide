use na::{self, Real};
use query::{PointProjection, PointQuery, PointQueryWithLocation};
use shape::{CompositeShape, FeatureId, Polyline, Triangle, SegmentPointLocation};
use bounding_volume::AABB;
use partitioning::{BVTCostFn, BVTVisitor};
use utils::IsometryOps;
use math::{Isometry, Point};


impl<N: Real> PointQuery<N> for Polyline<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        let (projection, _) = self.project_point_with_location(m, point, solid);
        projection
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, point: &Point<N>) -> (PointProjection<N>, FeatureId) {
        unimplemented!()
    }

    // FIXME: implement distance_to_point too?

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, point: &Point<N>) -> bool {
        let ls_pt = m.inverse_transform_point(point);
        let mut test = PointContainementTest {
            polyline: self,
            point: &ls_pt,
            found: false,
        };

        self.bvt().visit(&mut test);

        test.found
    }
}

impl<N: Real> PointQueryWithLocation<N> for Polyline<N> {
    type Location = (usize, SegmentPointLocation<N>);

    #[inline]
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
        solid: bool,
    ) -> (PointProjection<N>, Self::Location) {
        let ls_pt = m.inverse_transform_point(point);
        let mut cost_fn = PolylinePointProjCostFn {
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
struct PolylinePointProjCostFn<'a, N: 'a + Real> {
    polyline: &'a Polyline<N>,
    point: &'a Point<N>,
}

impl<'a, N: Real> BVTCostFn<N, usize, AABB<N>> for PolylinePointProjCostFn<'a, N> {
    type UserData = (PointProjection<N>, (usize, SegmentPointLocation<N>));

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        Some(aabb.distance_to_point(&Isometry::identity(), self.point, true))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, Self::UserData)> {
        let (proj, extra_info) =
            self.polyline
                .segment_at(*b)
                .project_point_with_location(&Isometry::identity(), self.point, true);

        let extra_info = (*b, extra_info);
        Some((na::distance(self.point, &proj.point), (proj, extra_info)))
    }
}

/*
 * Visitor.
 */
/// Bounding Volume Tree visitor collecting nodes that may contain a given point.
struct PointContainementTest<'a, N: 'a + Real> {
    polyline: &'a Polyline<N>,
    point: &'a Point<N>,
    found: bool,
}

impl<'a, N: Real> BVTVisitor<usize, AABB<N>> for PointContainementTest<'a, N> {
    #[inline]
    fn visit_internal(&mut self, bv: &AABB<N>) -> bool {
        !self.found && bv.contains_point(&Isometry::identity(), self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &usize, bv: &AABB<N>) {
        if !self.found && bv.contains_point(&Isometry::identity(), self.point)
            && self.polyline
                .segment_at(*b)
                .contains_point(&Isometry::identity(), self.point)
        {
            self.found = true;
        }
    }
}