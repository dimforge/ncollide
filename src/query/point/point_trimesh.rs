use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point};
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor};
use crate::query::{
    visitors::CompositePointContainmentTest, PointProjection, PointQuery, PointQueryWithLocation,
};
use crate::shape::{CompositeShape, FeatureId, TriMesh, TrianglePointLocation};
use na::{self, RealField};

impl<N: RealField> PointQuery<N> for TriMesh<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        let (projection, _) = self.project_point_with_location(m, point, solid);
        projection
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
    ) -> (PointProjection<N>, FeatureId) {
        let (projection, (triangle_id, location)) =
            self.project_point_with_location(m, point, false);
        let face = &self.faces()[triangle_id];
        let feature_id = match location {
            TrianglePointLocation::OnVertex(triangle_local_id) => {
                FeatureId::Vertex(face.indices[triangle_local_id])
            }
            TrianglePointLocation::OnEdge(triangle_local_id, _) => {
                FeatureId::Edge(face.edges[triangle_local_id])
            }
            TrianglePointLocation::OnFace(_, _) => FeatureId::Face(triangle_id),
            TrianglePointLocation::OnSolid => FeatureId::Unknown,
        };
        (projection, feature_id)
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

impl<N: RealField> PointQueryWithLocation<N> for TriMesh<N> {
    type Location = (usize, TrianglePointLocation<N>);

    #[inline]
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
        _: bool,
    ) -> (PointProjection<N>, Self::Location) {
        let ls_pt = m.inverse_transform_point(point);
        let mut visitor = TriMeshPointProjVisitor {
            polyline: self,
            point: &ls_pt,
        };

        let (mut proj, extra_info) = self.bvh().best_first_search(&mut visitor).unwrap().1;
        proj.point = m * proj.point;

        (proj, extra_info)
    }
}

/*
 * Visitors
 */
struct TriMeshPointProjVisitor<'a, N: 'a + RealField> {
    polyline: &'a TriMesh<N>,
    point: &'a Point<N>,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>> for TriMeshPointProjVisitor<'a, N> {
    type Result = (PointProjection<N>, (usize, TrianglePointLocation<N>));

    #[inline]
    fn visit(
        &mut self,
        best: N,
        aabb: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        let dist = aabb.distance_to_point(&Isometry::identity(), self.point, true);

        let mut res = BestFirstVisitStatus::Continue {
            cost: dist,
            result: None,
        };

        if let Some(b) = data {
            if dist < best {
                let (proj, extra_info) = self.polyline.triangle_at(*b).project_point_with_location(
                    &Isometry::identity(),
                    self.point,
                    true,
                );

                let extra_info = (*b, extra_info);
                res = BestFirstVisitStatus::Continue {
                    cost: na::distance(self.point, &proj.point),
                    result: Some((proj, extra_info)),
                };
            }
        }

        res
    }
}
