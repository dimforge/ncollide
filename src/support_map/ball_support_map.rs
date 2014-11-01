use na;
use na::Translation;
use support_map::{PreferedSamplingDirections, SupportMap};
use shape::Ball;
use math::{Scalar, Point, Vect};


impl<N, P, V, M> SupportMap<P, V, M> for Ball<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translation<V> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        na::orig::<P>() + m.translation() + na::normalize(dir) * self.radius()
    }
}

impl<N, V, M> PreferedSamplingDirections<V, M> for Ball<N> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
