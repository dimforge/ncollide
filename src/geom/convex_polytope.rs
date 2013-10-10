use std::num::Bounded;
use nalgebra::na::{Cast, Transform, Rotate, AlgebraicVec};
use geom::{Implicit, HasMargin};

/**
 * Set of point assumed to form a convex polytope.
 * 
 *   - `V`: type of the polytope points.
 *   - `N`: type of the result of a dot product between two points.
 */
pub struct ConvexPolytope<N, V> {
    priv pts:    ~[V],
    priv margin: N
}

impl<N: Cast<f32>, V> ConvexPolytope<N, V> {
    /**
     * Creates a polytope from a set of point. Those points are assumed to form
     * a convex polytope: convexity is not checked.
     */
    #[inline]
    pub fn new(pts: ~[V]) -> ConvexPolytope<N, V> {
        ConvexPolytope::new_with_margin(pts, Cast::from(0.04))
    }

    /**
     * Creates a polytope from a set of point and a custom margin. Those points are assumed to form
     * a convex polytope: convexity is not checked.
     */
    #[inline]
    pub fn new_with_margin(pts: ~[V], margin: N) -> ConvexPolytope<N, V> {
        ConvexPolytope {
            pts:    pts,
            margin: margin
        }
    }
}

impl<N: Clone, V> HasMargin<N> for ConvexPolytope<N, V> {
    #[inline]
    fn margin(&self) -> N {
        self.margin.clone()
    }
}

impl<N: Algebraic + Ord + Bounded + ToStr + Neg<N> + Clone,
     V: AlgebraicVec<N> + Clone,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for ConvexPolytope<N, V> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let _M: N = Bounded::max_value();
        let mut best_dot = -_M;
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
