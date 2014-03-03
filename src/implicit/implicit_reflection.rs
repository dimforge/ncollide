use implicit::{Implicit, HasMargin};
use geom::Reflection;
use math::{Scalar, Vector};

impl<'a, G: HasMargin> HasMargin for Reflection<'a, G> {
    #[inline]
    fn margin(&self) -> Scalar {
        self.g().margin()
    }
}

impl<'a, _M, G: Implicit<Vector, _M>> Implicit<Vector, _M> for Reflection<'a, G> {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vector) -> Vector {
        -self.g().support_point(m, &-dir)
    }

    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vector) -> Vector {
        -self.g().support_point_without_margin(m, &-dir)
    }
}
