use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point};
use na::{self, Real};
use crate::partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor};
use crate::query::{
    visitors::CompositePointContainmentTest, PointProjection, PointQuery, PointQueryWithLocation,
};
use crate::shape::{CompositeShape, FeatureId, TriMesh, TrianglePointLocation};
use crate::utils::IsometryOps;

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
    ) -> (PointProjection<N>, FeatureId)
    {
        unimplemented!()
    }

    // FIXME: implement distance_to_point too?

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, point: &Point<N>) -> bool {
        let ls_pt = m.inverse_transform_point(point);
        let mut visitor = CompositePointContainmentTest {
            shape: self,
            point: &ls_pt,
            found: false,
        };

        self.bvh().visit(&mut visitor);

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
    ) -> (PointProjection<N>, Self::Location)
    {
        let ls_pt = m.inverse_transform_point(point);
        let mut visitor = TriMeshPointProjVisitor {
            polyline: self,
            point: &ls_pt,
        };

        let (mut proj, extra_info) = self.bvh().best_first_search(&mut visitor).unwrap();
        proj.point = m * proj.point;

        (proj, extra_info)
    }
}

/*
 * Visitors
 */
struct TriMeshPointProjVisitor<'a, N: 'a + Real> {
    polyline: &'a TriMesh<N>,
    point: &'a Point<N>,
}

impl<'a, N: Real> BestFirstVisitor<N, usize, AABB<N>> for TriMeshPointProjVisitor<'a, N> {
    type Result = (PointProjection<N>, (usize, TrianglePointLocation<N>));

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        BestFirstBVVisitStatus::ContinueWithCost(aabb.distance_to_point(
            &Isometry::identity(),
            self.point,
            true,
        ))
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, Self::Result> {
        let (proj, extra_info) = self.polyline.triangle_at(*b).project_point_with_location(
            &Isometry::identity(),
            self.point,
            true,
        );

        let extra_info = (*b, extra_info);
        BestFirstDataVisitStatus::ContinueWithResult(
            na::distance(self.point, &proj.point),
            (proj, extra_info),
        )
    }
}
