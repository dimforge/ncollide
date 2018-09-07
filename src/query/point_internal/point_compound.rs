use bounding_volume::AABB;
use math::{Isometry, Point};
use na::{self, Real};
use partitioning::{BVTCostFn, Visitor, VisitStatus};
use query::{PointProjection, PointQuery, visitors::PointContainmentTest};
use shape::{CompositeShape, Compound, FeatureId};
use utils::IsometryOps;

impl<N: Real> PointQuery<N> for Compound<N> {
    // XXX: if solid == false, this might return internal projection.
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        let ls_pt = m.inverse_transform_point(point);
        let mut cost_fn = CompoundPointProjCostFn {
            compound: self,
            point: &ls_pt,
            solid: solid,
        };

        let mut proj = self.bvt().best_first_search(&mut cost_fn).unwrap().1;
        proj.point = m * proj.point;

        proj
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        _: &Isometry<N>,
        _: &Point<N>,
    ) -> (PointProjection<N>, FeatureId) {
        // XXX Properly propagate the feature id.
        unimplemented!()
        // (self.project_point(m, point), FeatureId::Unknown)
    }

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

/*
 * Costs function.
 */
struct CompoundPointProjCostFn<'a, N: 'a + Real> {
    compound: &'a Compound<N>,
    point: &'a Point<N>,
    solid: bool,
}

impl<'a, N: Real> BVTCostFn<N, usize, AABB<N>> for CompoundPointProjCostFn<'a, N> {
    type UserData = PointProjection<N>;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        Some(aabb.distance_to_point(&Isometry::identity(), self.point, true))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, PointProjection<N>)> {
        let mut res = None;

        self.compound.map_part_at(*b, &mut |_, objm, obj| {
            let proj = obj.project_point(objm, self.point, self.solid);

            res = Some((na::distance(self.point, &proj.point), proj));
        });

        res
    }
}