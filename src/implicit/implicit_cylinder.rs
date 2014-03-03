use std::num::Zero;
use nalgebra::na::{Indexable, Rotate, Transform, Norm};
use nalgebra::na;
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Cylinder;
use math::{Scalar, Vector};


impl HasMargin for Cylinder {
    #[inline]
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

impl<_M: Transform<Vector> + Rotate<Vector>>
Implicit<Vector, _M> for Cylinder {
    fn support_point_without_margin(&self, m: &_M, dir: &Vector) -> Vector {
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

impl<_M: Rotate<Vector>>
PreferedSamplingDirections<Vector, _M> for Cylinder {
    #[inline(always)]
    fn sample(&self, transform: &_M, f: |Vector| -> ()) {
        // Sample along the principal axis
        let mut v: Vector = na::zero();
        v.set(0, na::one());

        let rv = transform.rotate(&v);
        f(-rv);
        f(rv);
    }
}
