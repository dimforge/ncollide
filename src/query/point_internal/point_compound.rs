use alga::general::Id;
use na;
use query::{PointProjection, PointQuery};
use bounding_volume::AABB;
use shape::{CompositeShape, Compound, FeatureId};
use partitioning::{BVTCostFn, BVTVisitor};
use math::{Isometry, Point};

impl<N: Real> PointQuery<N> for Compound<N> {
    // XXX: if solid == false, this might return internal projection.
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &P, solid: bool) -> PointProjection<N> {
        let ls_pt = m.inverse_transform_point(point);
        let mut cost_fn = CompoundPointProjCostFn {
            compound: self,
            point: &ls_pt,
            solid: solid,
        };

        let mut proj = self.bvt().best_first_search(&mut cost_fn).unwrap().1;
        proj.point = m.transform_point(&proj.point);

        proj
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, point: &P) -> (PointProjection<N>, FeatureId) {
        // XXX Properly propagate the feature id.
        unimplemented!()
        // (self.project_point(m, point), FeatureId::Unknown)
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, point: &P) -> bool {
        let ls_pt = m.inverse_transform_point(point);
        let mut test = PointContainementTest {
            compound: self,
            point: &ls_pt,
            found: false,
        };

        self.bvt().visit(&mut test);

        test.found
    }
}

/*
 * Costs function.
 */
struct CompoundPointProjCostFn<'a, P: 'a + Point, M: 'a + Isometry<P>> {
    compound: &'a Compound<N>,
    point: &'a P,
    solid: bool,
}

impl<'a, P, M> BVTCostFn<N, usize, AABB<N>> for CompoundPointProjCostFn<'a, P, M>
where
    N: Real,
    M: Isometry<P>,
{
    type UserData = PointProjection<N>;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        Some(aabb.distance_to_point(&Id::new(), self.point, true))
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

/*
 * Visitor.
 */
/// Bounding Volume Tree visitor collecting nodes that may contain a given point.
struct PointContainementTest<'a, P: 'a + Point, M: 'a + Isometry<P>> {
    compound: &'a Compound<N>,
    point: &'a P,
    found: bool,
}

impl<'a, P, M> BVTVisitor<usize, AABB<N>> for PointContainementTest<'a, P, M>
where
    N: Real,
    M: Isometry<P>,
{
    #[inline]
    fn visit_internal(&mut self, bv: &AABB<N>) -> bool {
        !self.found && bv.contains_point(&Id::new(), self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &usize, bv: &AABB<N>) {
        if !self.found && bv.contains_point(&Id::new(), self.point) {
            self.compound.map_part_at(*b, &mut |_, objm, obj| {
                self.found = obj.contains_point(objm, self.point)
            })
        }
    }
}
