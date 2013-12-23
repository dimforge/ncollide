use nalgebra::na::AlgebraicVec;
use implicit::{Implicit, HasMargin};
use geom::Reflection;

impl<'a, N, G: HasMargin<N>> HasMargin<N> for Reflection<'a, G> {
    #[inline]
    fn margin(&self) -> N {
        self.g().margin()
    }
}

impl<'a, N: Algebraic, V: AlgebraicVec<N>, M, G: Implicit<N, V, M>> Implicit<N, V, M> for Reflection<'a, G> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> V {
        -self.g().support_point(m, &-dir)
    }

    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        -self.g().support_point_without_margin(m, &-dir)
    }
}
