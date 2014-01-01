use std::num::Zero;
use nalgebra::na::{Indexable, Rotate, Transform, Norm};
use nalgebra::na;
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Cone;
use math::{N, V};

impl HasMargin for Cone {
    #[inline]
    fn margin(&self) -> N {
        self.margin()
    }
}

impl<_M: Transform<V> + Rotate<V>>
Implicit<V, _M> for Cone {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        vres.set(0, na::zero());

        if vres.normalize().is_zero() {
            vres = na::zero();

            if local_dir.at(0).is_negative() {
                vres.set(0, -self.half_height())
            }
            else {
                vres.set(0, self.half_height())
            }
        }
        else {
            vres = vres * self.radius();
            vres.set(0, -self.half_height());

            if na::dot(&local_dir, &vres) < local_dir.at(0) * self.half_height() {
                vres = na::zero();
                vres.set(0, self.half_height())
            }
        }

        m.transform(&vres)
    }
}

impl<_M: Rotate<V>>
PreferedSamplingDirections<V, _M> for Cone {
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
