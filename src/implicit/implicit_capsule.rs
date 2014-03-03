use std::num::Zero;
use nalgebra::na::{Indexable, Rotate, Transform};
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Capsule;
use math::{Scalar, Vector};

impl HasMargin for Capsule {
    #[inline]
    fn margin(&self) -> Scalar {
        self.radius().clone()
    }
}

impl<_M: Transform<Vector> + Rotate<Vector>>
Implicit<Vector, _M> for Capsule {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vector) -> Vector {
        let local_dir = m.inv_rotate(dir);

        let mut vres: Vector = Zero::zero();

        if local_dir.at(0).is_negative() {
            vres.set(0, -self.half_height())
        }
        else {
            vres.set(0, self.half_height())
        }

        m.transform(&vres)
    }
}

impl<Vector, _M>
PreferedSamplingDirections<Vector, _M> for Capsule {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |Vector| -> ()) {
    }
}
