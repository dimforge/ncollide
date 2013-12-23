use std::num::Zero;
use nalgebra::na::{Indexable, AlgebraicVecExt, Rotate, Transform};
use implicit::{Implicit, HasMargin};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;
use geom::Capsule;

impl<N: Clone> HasMargin<N> for Capsule<N> {
    #[inline]
    fn margin(&self) -> N {
        self.radius().clone()
    }
}

impl<N: Clone + Signed + Algebraic,
     V: Clone + AlgebraicVecExt<N>,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for Capsule<N> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres: V = Zero::zero();

        if local_dir.at(0).is_negative() {
            vres.set(0, -self.half_height())
        }
        else {
            vres.set(0, self.half_height())
        }

        m.transform(&vres)
    }
}

impl<N, V, M>
PreferedSamplingDirections<V, M> for Capsule<N> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
