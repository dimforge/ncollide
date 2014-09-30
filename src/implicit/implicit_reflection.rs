use implicit::Implicit;
use geom::Reflection;
use math::Vect;

impl<'a, _M, G: Implicit<Vect, _M>> Implicit<Vect, _M> for Reflection<'a, G> {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Vect {
        -self.geom().support_point(m, &-dir)
    }
}
