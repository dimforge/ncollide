use std::num::Bounded;
use nalgebra::na::{Transform, Rotate, AlgebraicVec};
use nalgebra::na;
use implicit::{Implicit, HasMargin};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;
use geom::Convex;

impl<N: Clone, V> HasMargin<N> for Convex<N, V> {
    #[inline]
    fn margin(&self) -> N {
        self.margin()
    }
}

impl<N: Algebraic + Ord + Bounded + Neg<N> + Clone,
     V: AlgebraicVec<N> + Clone,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for Convex<N, V> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let _M: N = Bounded::max_value();
        let mut best_dot = -_M;
        let mut best_pt  = &self.pts()[0];

        for p in self.pts().iter() {
            let dot = na::dot(p, &local_dir);

            if dot > best_dot {
                best_dot = dot;
                best_pt  = p;
            }
        }


        m.transform(best_pt)
    }
}

impl<N, V, M>
PreferedSamplingDirections<V, M> for Convex<N, V> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
