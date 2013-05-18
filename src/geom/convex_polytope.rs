use core::num::Bounded;
use nalgebra::traits::dot::Dot;
use geom::implicit::Implicit;

pub struct ConvexPolytope<V, T>
{
  pts: ~[V]
}

pub fn convex_polytope<V: Dot<T>, T>(pts: ~[V]) -> ConvexPolytope<V, T>
{ ConvexPolytope { pts: pts } }

impl<T: Ord + Bounded + ToStr + Neg<T>, V: Dot<T> + Copy>
Implicit<V> for ConvexPolytope<V, T>
{
  fn support_point(&self, dir: &V) -> V
  {
    let mut best_dot : T = -Bounded::max_value::<T>();
    let mut best_pt  : &V = &self.pts[0];

    for self.pts.each |p|
    {
      let dot = p.dot(dir);

      if (dot > best_dot)
      {
        best_dot = dot;
        best_pt  = p;
      }
    }


    copy *best_pt
  }
}
