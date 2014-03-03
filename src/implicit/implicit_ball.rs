use nalgebra::na::Translation;
use implicit::{PreferedSamplingDirections, Implicit, HasMargin};
use geom::Ball;
use math::{Scalar, Vector};

impl HasMargin for Ball {
    #[inline]
    fn margin(&self) -> Scalar {
        self.radius()
    }
}

impl<_M: Translation<Vector>> Implicit<Vector, _M> for Ball {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, _: &Vector) -> Vector {
        m.translation()
    }
}

impl<_M> PreferedSamplingDirections<Vector, _M> for Ball {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |Vector| -> ()) {
    }
}
