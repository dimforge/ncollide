use std::num::Zero;
use nalgebra::na::{Indexable, Rotate, Transform};
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Capsule;
use math::{N, V};

impl HasMargin for Capsule {
    #[inline]
    fn margin(&self) -> N {
        self.radius().clone()
    }
}

impl<_M: Transform<V> + Rotate<V>>
Implicit<V, _M> for Capsule {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres: V = Zero::zero();

        if local_dir.at(0).is_negative() {
            vres.set(0, -self.half_height())
        }
        else {
            vres.set(0, self.half_height())
        }

        m.transform(&vres)
    }
}

impl<V, _M>
PreferedSamplingDirections<V, _M> for Capsule {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |V| -> ()) {
    }
}
