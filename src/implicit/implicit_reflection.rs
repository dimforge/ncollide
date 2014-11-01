use implicit::Implicit;
use shape::Reflection;
use math::{Scalar, Vect};


impl<'a, N, P, V, M, G> Implicit<P, V, M> for Reflection<'a, G>
    where N: Scalar,
          P: Neg<P>,
          V: Vect<N>,
          G: Implicit<P, V, M> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        -self.geom().support_point(m, &-*dir)
    }
}
