
use std::num::Signed;
use na::{Transform, Rotate, Basis};
use na;
use geom::Cuboid;
use implicit::{Implicit, PreferedSamplingDirections};
use math::{Scalar, Point, Vect};


impl<N, P, V, M> Implicit<P, V, M> for Cuboid<V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Rotate<V> + Transform<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        let local_dir = m.inv_rotate(dir);

        let mut pres: P = na::orig();

        let he = self.half_extents();
        for i in range(0u, na::dim::<P>()) {
            if local_dir[i].is_negative() {
                pres[i] = -(*he)[i];
            }
            else {
                pres[i] = (*he)[i];
            }
        }

        m.transform(&pres)
    }
}

impl<V, M> PreferedSamplingDirections<V, M> for Cuboid<V>
    where V: Basis + Neg<V>,
          M: Rotate<V> {
    #[inline(always)]
    fn sample(&self, transform: &M, f: |V| -> ()) {
        na::canonical_basis(|e: V| {
            let re = transform.rotate(&e);

            f(-re);
            f(re);

            true
        })
    }
}
