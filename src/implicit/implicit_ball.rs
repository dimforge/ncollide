use nalgebra::na::{AlgebraicVec, Translation};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;
use implicit::{Implicit, HasMargin};
use geom::Ball;

impl<N: Clone> HasMargin<N> for Ball<N> {
    #[inline]
    fn margin(&self) -> N {
        self.radius()
    }
}

impl<N: Algebraic + Clone, V: AlgebraicVec<N>, M: Translation<V>> Implicit<N, V, M> for Ball<N> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, _: &V) -> V {
        m.translation()
    }
}

impl<N, V, M> PreferedSamplingDirections<V, M> for Ball<N> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
