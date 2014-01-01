use nalgebra::na::Translation;
use implicit::{PreferedSamplingDirections, Implicit, HasMargin};
use geom::Ball;
use math::{N, V};

impl HasMargin for Ball {
    #[inline]
    fn margin(&self) -> N {
        self.radius()
    }
}

impl<_M: Translation<V>> Implicit<V, _M> for Ball {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, _: &V) -> V {
        m.translation()
    }
}

impl<_M> PreferedSamplingDirections<V, _M> for Ball {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |V| -> ()) {
    }
}
