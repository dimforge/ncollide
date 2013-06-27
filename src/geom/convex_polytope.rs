use std::num::Bounded;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::transformation::Transformable;
use geom::transformed::Transformed;
use geom::implicit::Implicit;

/**
 * Set of point assumed to form a convex polytope.
 * 
 *   - `V`: type of the polytope points.
 *   - `N`: type of the result of a dot product between two points.
 */
pub struct ConvexPolytope<V, N>
{ priv pts: @[V] }

impl<V, N> ConvexPolytope<V, N>
{
  /**
   * Creates a polytope from a set of point. Those points are assumed to form
   * a convex polytope: convexity is not checked.
   */
  #[inline(always)]
  pub fn new(pts: @[V]) -> ConvexPolytope<V, N>
  { ConvexPolytope { pts: pts } }
}

impl<N: Ord + Bounded + ToStr + Neg<N>, V: Dot<N> + Copy>
Implicit<V> for ConvexPolytope<V, N>
{
  #[inline]
  fn support_point(&self, dir: &V) -> V
  {
    let mut best_dot = -Bounded::max_value::<N>();
    let mut best_pt  = &self.pts[0];

    for self.pts.iter().advance |p|
    {
      let dot = p.dot(dir);

      if dot > best_dot
      {
        best_dot = dot;
        best_pt  = p;
      }
    }


    copy *best_pt
  }
}

impl<V: Copy, N, M: Copy> Transformable<M, Transformed<ConvexPolytope<V, N>, M>> for ConvexPolytope<V, N>
{
  fn transformed(&self, transform: &M) -> Transformed<ConvexPolytope<V, N>, M>
  { Transformed::new(copy *transform, copy *self) }
}
