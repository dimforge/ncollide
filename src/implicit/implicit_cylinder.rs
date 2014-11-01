use std::num::Zero;
use na::{Rotate, Transform, Norm};
use na;
use implicit::{Implicit, PreferedSamplingDirections};
use shape::Cylinder;
use math::{Scalar, Point, Vect};



impl<N, P, V, M> Implicit<P, V, M> for Cylinder<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    fn support_point(&self, m: &M, dir: &V) -> P {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir[1].is_negative();

        vres[1]  = na::zero();

        if vres.normalize().is_zero() {
            vres = na::zero()
        }
        else {
            vres = vres * self.radius();
        }

        if negative {
            vres[1] = -self.half_height()
        }
        else {
            vres[1] = self.half_height()
        }

        m.transform(&(na::orig::<P>() + vres))
    }
}

impl<N, V, M> PreferedSamplingDirections<V, M> for Cylinder<N>
    where N: Scalar,
          V: Vect<N>,
          M: Rotate<V> {
    #[inline(always)]
    fn sample(&self, transform: &M, f: |V| -> ()) {
        // Sample along the principal axis
        let mut v = na::zero::<V>();
        v[1] = na::one();

        let rv = transform.rotate(&v);
        f(-rv);
        f(rv);
    }
}
