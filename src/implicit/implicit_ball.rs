use nalgebra::na;
use nalgebra::na::Translation;
use implicit::{PreferedSamplingDirections, Implicit};
use geom::Ball;
use math::{Scalar, Vect};

impl<_M: Translation<Vect>> Implicit<Vect, _M> for Ball {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Vect {
        m.translation() + na::normalize(dir) * self.radius()
    }
}

impl<_M> PreferedSamplingDirections<Vect, _M> for Ball {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |Vect| -> ()) {
    }
}
