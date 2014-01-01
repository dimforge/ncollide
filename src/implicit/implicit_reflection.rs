use implicit::{Implicit, HasMargin};
use geom::Reflection;
use math::{N, V};

impl<'a, G: HasMargin> HasMargin for Reflection<'a, G> {
    #[inline]
    fn margin(&self) -> N {
        self.g().margin()
    }
}

impl<'a, _M, G: Implicit<V, _M>> Implicit<V, _M> for Reflection<'a, G> {
    #[inline]
    fn support_point(&self, m: &_M, dir: &V) -> V {
        -self.g().support_point(m, &-dir)
    }

    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &V) -> V {
        -self.g().support_point_without_margin(m, &-dir)
    }
}
