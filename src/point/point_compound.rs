use na::{Transform, Translate, AbsoluteRotate, Rotate};
use na;
use point::{LocalPointQuery, PointQuery};
use shape::{ConcaveShape, Compound};
use bounding_volume::AABB;
use partitioning::BVTCostFn;
use math::{Scalar, Point, Vect};


// XXX: if solid == false, this might return internal projection.
impl<N, P, V, M> LocalPointQuery<N, P> for Compound<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Send + Sync + AbsoluteRotate<V> + Transform<P> + Rotate<V> + Mul<M, M> + Clone {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        let mut cost_fn = CompoundPointProjCostFn { compound: self, point: point, solid: solid };

        self.bvt().best_first_search(&mut cost_fn).unwrap().val1()
    }

    #[inline]
    fn distance_to_point(&self, point: &P) -> N {
        na::dist(point, &self.project_point(point, true))
    }

    #[inline]
    fn contains_point(&self, point: &P) -> bool {
        na::approx_eq(&self.distance_to_point(point), &na::zero())
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for Compound<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Send + Sync + AbsoluteRotate<V> + Transform<P> + Rotate<V> + Mul<M, M> + Clone {
}


/*
 * Costs function.
 */
struct CompoundPointProjCostFn<'a, N: 'a, P: 'a, V: 'a, M: 'a> {
    compound: &'a Compound<N, P, V, M>,
    point:    &'a P,
    solid:    bool
}

impl<'a, N, P, V, M> BVTCostFn<N, uint, AABB<P>, P> for CompoundPointProjCostFn<'a, N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Send + Sync + AbsoluteRotate<V> + Transform<P> + Rotate<V> + Mul<M, M> + Clone {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        Some(aabb.distance_to_point(self.point))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, P)> {
        self.compound.map_part_at(*b, |objm, obj| {
            let proj = obj.project_point_with_transform(objm, self.point, self.solid);

            Some((na::dist(self.point, &proj), proj))
        })
    }
}
