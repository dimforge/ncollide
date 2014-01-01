use std::num::Zero;
use nalgebra::na::{Indexable, Rotate, Transform, Norm};
use nalgebra::na;
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Cylinder;
use math::{N, V};


impl HasMargin for Cylinder {
    #[inline]
    fn margin(&self) -> N {
        self.margin()
    }
}

impl<_M: Transform<V> + Rotate<V>>
Implicit<V, _M> for Cylinder {
    fn support_point_without_margin(&self, m: &_M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir.at(0).is_negative();

        vres.set(0, Zero::zero());

        if vres.normalize().is_zero() {
            vres = Zero::zero()
        }
        else {
            vres = vres * self.radius();
        }

        if negative {
            vres.set(0, -self.half_height())
        }
        else {
            vres.set(0, self.half_height())
        }

        m.transform(&vres)
    }
}

impl<_M: Rotate<V>>
PreferedSamplingDirections<V, _M> for Cylinder {
    #[inline(always)]
    fn sample(&self, transform: &_M, f: |V| -> ()) {
        // Sample along the principal axis
        let mut v: V = na::zero();
        v.set(0, na::one());

        let rv = transform.rotate(&v);
        f(-rv);
        f(rv);
    }
}
