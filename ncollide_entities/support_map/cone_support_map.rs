use na::{Rotate, Transform, Norm};
use na;
use support_map::{SupportMap, PreferedSamplingDirections};
use shape::Cone;
use math::{Scalar, Point, Vect};


impl<N, P, V, M> SupportMap<P, V, M> for Cone<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        vres[1] = na::zero();

        if na::is_zero(&vres.normalize()) {
            vres = na::zero();

            if local_dir[1].is_negative() {
                vres[1] = -self.half_height()
            }
            else {
                vres[1] = self.half_height()
            }
        }
        else {
            vres = vres * self.radius();
            vres[1] = -self.half_height();

            if na::dot(&local_dir, &vres) < local_dir[1] * self.half_height() {
                vres = na::zero();
                vres[1] = self.half_height()
            }
        }

        m.transform(&(na::orig::<P>() + vres))
    }
}

impl<N, V, M> PreferedSamplingDirections<V, M> for Cone<N>
    where N: Scalar,
          V: Vect<N>,
          M: Rotate<V> {
    #[inline(always)]
    fn sample(&self, transform: &M, f: |V| -> ()) {
        // Sample along the principal axis
        let mut v: V = na::zero();
        v[1] = na::one();

        let rv = transform.rotate(&v);
        f(-rv);
        f(rv);
    }
}
