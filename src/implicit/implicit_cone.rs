use std::num::Zero;
use nalgebra::na::{Indexable, Rotate, Transform, Norm};
use nalgebra::na;
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Cone;
use math::{Scalar, Vector};

impl HasMargin for Cone {
    #[inline]
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

impl<_M: Transform<Vector> + Rotate<Vector>>
Implicit<Vector, _M> for Cone {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vector) -> Vector {
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

impl<_M: Rotate<Vector>>
PreferedSamplingDirections<Vector, _M> for Cone {
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
