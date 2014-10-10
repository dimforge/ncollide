use implicit::Implicit;
use geom::Reflection;
use math::{Point, Vect};

impl<'a, _M, G: Implicit<Point, Vect, _M>> Implicit<Point, Vect, _M> for Reflection<'a, G> {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Point {
        -self.geom().support_point(m, &-dir)
    }
}
