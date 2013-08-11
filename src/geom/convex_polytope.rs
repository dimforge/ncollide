use std::num::Bounded;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::rotation::Rotate;
use geom::implicit::Implicit;

/**
 * Set of point assumed to form a convex polytope.
 * 
 *   - `V`: type of the polytope points.
 *   - `N`: type of the result of a dot product between two points.
 */
pub struct ConvexPolytope<N, V> {
    priv pts: @[V]
}

impl<N, V> ConvexPolytope<N, V> {
    /**
     * Creates a polytope from a set of point. Those points are assumed to form
     * a convex polytope: convexity is not checked.
     */
    #[inline]
    pub fn new(pts: @[V]) -> ConvexPolytope<N, V> {
        ConvexPolytope { pts: pts }
    }
}

impl<N: Ord + Bounded + ToStr + Neg<N>,
     V: Dot<N> + Clone,
     M: Transform<V> + Rotate<V>>
Implicit<V, M> for ConvexPolytope<N, V> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut best_dot = -Bounded::max_value::<N>();
        let mut best_pt  = &self.pts[0];

        for p in self.pts.iter() {
            let dot = p.dot(&local_dir);

            if dot > best_dot {
                best_dot = dot;
                best_pt  = p;
            }
        }


        m.transform(best_pt)
    }
}
