use nalgebra::na::Translation;
use implicit::{PreferedSamplingDirections, Implicit, HasMargin};
use geom::Ball;
use math::{Scalar, Vect};

impl HasMargin for Ball {
    #[inline]
    fn margin(&self) -> Scalar {
        self.radius()
    }
}

impl<_M: Translation<Vect>> Implicit<Vect, _M> for Ball {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, _: &Vect) -> Vect {
        m.translation()
    }
}

impl<_M> PreferedSamplingDirections<Vect, _M> for Ball {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |Vect| -> ()) {
    }
}
