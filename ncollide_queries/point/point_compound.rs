use na::Translate;
use na;
use point::{LocalPointQuery, PointQuery};
use entities::bounding_volume::AABB;
use entities::shape::{Compound, CompositeShape};
use entities::partitioning::{BVTCostFn, BVTVisitor};
use math::{Scalar, Point, Vect, Isometry};


// XXX: if solid == false, this might return internal projection.
impl<P, M> LocalPointQuery<P> for Compound<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        let mut cost_fn = CompoundPointProjCostFn { compound: self, point: point, solid: solid };

        self.bvt().best_first_search(&mut cost_fn).unwrap().1
    }

    #[inline]
    fn distance_to_point(&self, point: &P) -> <P::Vect as Vect>::Scalar {
        na::dist(point, &self.project_point(point, true))
    }

    #[inline]
    fn contains_point(&self, point: &P) -> bool {
        let mut test = PointContainementTest { compound: self, point: point, found: false };

        self.bvt().visit(&mut test);

        test.found
    }
}

impl<P, M> PointQuery<P, M> for Compound<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
}


/*
 * Costs function.
 */
struct CompoundPointProjCostFn<'a, P: 'a + Point, M: 'a> {
    compound: &'a Compound<P, M>,
    point:    &'a P,
    solid:    bool
}

impl<'a, P, M> BVTCostFn<<P::Vect as Vect>::Scalar, usize, AABB<P>, P> for CompoundPointProjCostFn<'a, P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<<P::Vect as Vect>::Scalar> {
        Some(aabb.distance_to_point(self.point))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(<P::Vect as Vect>::Scalar, P)> {
        let mut res = None;

        self.compound.map_part_at(*b, &mut |objm, obj| {
            let proj = obj.project_point_with_transform(objm, self.point, self.solid);

            res = Some((na::dist(self.point, &proj), proj));
        });

        res
    }
}

/*
 * Visitor.
 */
/// Bounding Volume Tree visitor collecting nodes that may contain a given point.
struct PointContainementTest<'a, P: 'a + Point, M: 'a> {
    compound: &'a Compound<P, M>,
    point:    &'a P,
    found:    bool
}

impl<'a, P, M> BVTVisitor<usize, AABB<P>> for PointContainementTest<'a, P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    #[inline]
    fn visit_internal(&mut self, bv: &AABB<P>) -> bool {
        !self.found && bv.contains_point(self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &usize, bv: &AABB<P>) {
        if !self.found && bv.contains_point(self.point) {
            self.compound.map_part_at(*b, &mut |objm, obj| {
                self.found = obj.contains_point_with_transform(objm, self.point)
            })
        }
    }
}
