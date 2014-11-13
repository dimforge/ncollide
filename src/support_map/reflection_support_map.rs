use support_map::SupportMap;
use shape::Reflection;
use math::{Scalar, Vect};


impl<'a, N, P, V, M, G> SupportMap<P, V, M> for Reflection<'a, G>
    where N: Scalar,
          P: Neg<P>,
          V: Vect<N>,
          G: SupportMap<P, V, M> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        -self.shape().support_point(m, &-*dir)
    }
}
