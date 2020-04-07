use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point};
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor, BVH};
use crate::query::{
    visitors::CompositePointContainmentTest, PointProjection, PointQuery, PointQueryWithLocation,
};
use crate::shape::{FeatureId, Polyline, SegmentPointLocation};
use na::{self, RealField};

impl<N: RealField> PointQuery<N> for Polyline<N> {
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
        let ls_pt = m.inverse_transform_point(point);
        let mut visitor = PolylinePointProjWithFeatureVisitor {
            polyline: self,
            point: &ls_pt,
        };

        let (mut proj, (id, feature)) = self.bvt().best_first_search(&mut visitor).unwrap().1;
        proj.point = m * proj.point;

        let polyline_feature = self.segment_feature_to_polyline_feature(id, feature);

        (proj, polyline_feature)
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

        self.bvt().visit(&mut visitor);

        visitor.found
    }
}

impl<N: RealField> PointQueryWithLocation<N> for Polyline<N> {
    type Location = (usize, SegmentPointLocation<N>);

    #[inline]
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
        _: bool,
    ) -> (PointProjection<N>, Self::Location) {
        let ls_pt = m.inverse_transform_point(point);
        let mut visitor = PolylinePointProjVisitor {
            polyline: self,
            point: &ls_pt,
        };

        let (mut proj, extra_info) = self.bvt().best_first_search(&mut visitor).unwrap().1;
        proj.point = m * proj.point;

        (proj, extra_info)
    }
}

/*
 * Visitors
 */
macro_rules! gen_visitor(
    ($Visitor: ident, $Location: ty, $project: ident $(, $args: ident)*) => {
        struct $Visitor<'a, N: 'a + RealField> {
            polyline: &'a Polyline<N>,
            point: &'a Point<N>,
        }

        impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>> for $Visitor<'a, N> {
            type Result = (PointProjection<N>, (usize, $Location));

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
                        let (proj, extra_info) = self.polyline.segment_at(*b).$project(
                            &Isometry::identity(),
                            self.point
                            $(, $args)*
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
    }
);

gen_visitor!(
    PolylinePointProjVisitor,
    SegmentPointLocation<N>,
    project_point_with_location,
    true
);
gen_visitor!(
    PolylinePointProjWithFeatureVisitor,
    FeatureId,
    project_point_with_feature
);
