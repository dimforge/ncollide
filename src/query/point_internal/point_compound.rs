use bounding_volume::AABB;
use math::{Isometry, Point};
use na::{self, Real};
use partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor, BVH};
use query::{visitors::CompositePointContainmentTest, PointProjection, PointQuery};
use shape::{CompositeShape, Compound, FeatureId};
use utils::IsometryOps;

impl<N: Real> PointQuery<N> for Compound<N> {
    // XXX: if solid == false, this might return internal projection.
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        let ls_pt = m.inverse_transform_point(point);
        let mut visitor = CompoundPointProjVisitor {
            compound: self,
            point: &ls_pt,
            solid: solid,
        };

        let mut proj = self.bvt().best_first_search(&mut visitor).unwrap();
        proj.point = m * proj.point;

        proj
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        _: &Isometry<N>,
        _: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
        // XXX Properly propagate the feature id.
        unimplemented!()
        // (self.project_point(m, point), FeatureId::Unknown)
    }

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

/*
 * Visitors
 */
struct CompoundPointProjVisitor<'a, N: 'a + Real> {
    compound: &'a Compound<N>,
    point: &'a Point<N>,
    solid: bool,
}

impl<'a, N: Real> BestFirstVisitor<N, usize, AABB<N>> for CompoundPointProjVisitor<'a, N> {
    type Result = PointProjection<N>;

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        BestFirstBVVisitStatus::ContinueWithCost(aabb.distance_to_point(
            &Isometry::identity(),
            self.point,
            true,
        ))
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, PointProjection<N>> {
        let mut res = BestFirstDataVisitStatus::Continue;

        self.compound.map_part_at(*b, &Isometry::identity(), &mut |objm, obj| {
            let proj = obj.project_point(objm, self.point, self.solid);

            res = BestFirstDataVisitStatus::ContinueWithResult(
                na::distance(self.point, &proj.point),
                proj,
            );
        });

        res
    }
}
