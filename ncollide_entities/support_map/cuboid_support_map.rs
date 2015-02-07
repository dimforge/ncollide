use std::ops::Neg;
use na::{Transform, Rotate, Basis};
use na;
use shape::Cuboid;
use support_map::{SupportMap, PreferedSamplingDirections};
use math::{Scalar, Point, Vect};


#[old_impl_check]
impl<N, P, V, M> SupportMap<P, V, M> for Cuboid<V>
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
            if local_dir[i] < na::zero() {
                pres[i] = -he[i];
            }
            else {
                pres[i] = he[i];
            }
        }

        m.transform(&pres)
    }
}

impl<V: Clone, M> PreferedSamplingDirections<V, M> for Cuboid<V>
    where V: Basis + Neg<Output = V>,
          M: Rotate<V> {
    #[inline(always)]
    fn sample(&self, transform: &M, f: &mut FnMut(V)) {
        na::canonical_basis(|e: V| {
            let re = transform.rotate(&e);

            f(-re.clone());
            f(re);

            true
        })
    }
}
