use alga::general::Id;
use na;
use query::{PointProjection, PointQuery};
use bounding_volume::AABB;
use shape::{CompositeShape, Compound, FeatureId};
use partitioning::{BVTCostFn, BVTVisitor};
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Compound<P, M> {
    // XXX: if solid == false, this might return internal projection.
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
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
    fn project_point_with_feature(&self, m: &M, point: &P) -> (PointProjection<P>, FeatureId) {
        // XXX Properly propagate the feature id.
        unimplemented!()
        // (self.project_point(m, point), FeatureId::Unknown)
    }

    #[inline]
    fn contains_point(&self, m: &M, point: &P) -> bool {
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
    compound: &'a Compound<P, M>,
    point: &'a P,
    solid: bool,
}

impl<'a, P, M> BVTCostFn<P::Real, usize, AABB<P>> for CompoundPointProjCostFn<'a, P, M>
where
    P: Point,
    M: Isometry<P>,
{
    type UserData = PointProjection<P>;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<P::Real> {
        Some(aabb.distance_to_point(&Id::new(), self.point, true))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(P::Real, PointProjection<P>)> {
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
    compound: &'a Compound<P, M>,
    point: &'a P,
    found: bool,
}

impl<'a, P, M> BVTVisitor<usize, AABB<P>> for PointContainementTest<'a, P, M>
where
    P: Point,
    M: Isometry<P>,
{
    #[inline]
    fn visit_internal(&mut self, bv: &AABB<P>) -> bool {
        !self.found && bv.contains_point(&Id::new(), self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &usize, bv: &AABB<P>) {
        if !self.found && bv.contains_point(&Id::new(), self.point) {
            self.compound.map_part_at(*b, &mut |_, objm, obj| {
                self.found = obj.contains_point(objm, self.point)
            })
        }
    }
}
