use std::num::Zero;
use nalgebra::na::{Indexable, Rotate, Transform};
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Capsule;
use math::{Scalar, Vect};

impl HasMargin for Capsule {
    #[inline]
    fn margin(&self) -> Scalar {
        self.radius().clone()
    }
}

impl<_M: Transform<Vect> + Rotate<Vect>>
Implicit<Vect, _M> for Capsule {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vect) -> Vect {
        let local_dir = m.inv_rotate(dir);

        let mut vres: Vect = Zero::zero();

        if local_dir.at(1).is_negative() {
            vres.set(1, -self.half_height())
        }
        else {
            vres.set(1, self.half_height())
        }

        m.transform(&vres)
    }
}

impl<Vect, _M>
PreferedSamplingDirections<Vect, _M> for Capsule {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |Vect| -> ()) {
    }
}
