use na::Translate;
use na;
use point::{LocalPointQuery, PointQuery};
use entities::bounding_volume::AABB;
use entities::shape::{Compound, CompositeShape};
use entities::partitioning::{BVTCostFn, BVTVisitor};
use math::{Scalar, Point, Vect, Isometry};


// XXX: if solid == false, this might return internal projection.
impl<N, P, V, M> LocalPointQuery<N, P> for Compound<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        let mut cost_fn = CompoundPointProjCostFn { compound: self, point: point, solid: solid };

        self.bvt().best_first_search(&mut cost_fn).unwrap().1
    }

    #[inline]
    fn distance_to_point(&self, point: &P) -> N {
        na::dist(point, &self.project_point(point, true))
    }

    #[inline]
    fn contains_point(&self, point: &P) -> bool {
        let mut test = PointContainementTest { compound: self, point: point, found: false };

        self.bvt().visit(&mut test);

        test.found
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for Compound<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
}


/*
 * Costs function.
 */
struct CompoundPointProjCostFn<'a, N: 'a, P: 'a, V: 'a, M: 'a> {
    compound: &'a Compound<N, P, V, M>,
    point:    &'a P,
    solid:    bool
}

impl<'a, N, P, V, M> BVTCostFn<N, usize, AABB<P>, P> for CompoundPointProjCostFn<'a, N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        Some(aabb.distance_to_point(self.point))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, P)> {
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
struct PointContainementTest<'a, N: 'a, P: 'a, V: 'a, M: 'a> {
    compound: &'a Compound<N, P, V, M>,
    point:    &'a P,
    found:    bool
}

impl<'a, N, P, V, M> BVTVisitor<usize, AABB<P>> for PointContainementTest<'a, N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
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
