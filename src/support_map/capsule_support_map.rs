use na;
use na::{Rotate, Transform};
use support_map::{SupportMap, PreferedSamplingDirections};
use shape::Capsule;
use math::{Scalar, Point, Vect};


impl<N, P, V, M> SupportMap<P, V, M> for Capsule<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        let local_dir = m.inv_rotate(dir);

        let mut pres = na::orig::<P>();

        if local_dir[1].is_negative() {
            pres[1] = -self.half_height()
        }
        else {
            pres[1] = self.half_height()
        }

        m.transform(&(pres + na::normalize(&local_dir) * self.radius()))
    }
}

impl<N, V, M> PreferedSamplingDirections<V, M> for Capsule<N> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
