use std::num::Zero;
use nalgebra::na::{Indexable, Rotate, Transform, Norm};
use nalgebra::na;
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Cylinder;
use math::{Scalar, Vect};


impl HasMargin for Cylinder {
    #[inline]
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

impl<_M: Transform<Vect> + Rotate<Vect>>
Implicit<Vect, _M> for Cylinder {
    fn support_point_without_margin(&self, m: &_M, dir: &Vect) -> Vect {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir.at(1).is_negative();

        vres.set(1, Zero::zero());

        if vres.normalize().is_zero() {
            vres = Zero::zero()
        }
        else {
            vres = vres * self.radius();
        }

        if negative {
            vres.set(1, -self.half_height())
        }
        else {
            vres.set(1, self.half_height())
        }

        m.transform(&vres)
    }
}

impl<_M: Rotate<Vect>>
PreferedSamplingDirections<Vect, _M> for Cylinder {
    #[inline(always)]
    fn sample(&self, transform: &_M, f: |Vect| -> ()) {
        // Sample along the principal axis
        let mut v: Vect = na::zero();
        v.set(1, na::one());

        let rv = transform.rotate(&v);
        f(-rv);
        f(rv);
    }
}
