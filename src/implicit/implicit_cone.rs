use std::num::Zero;
use na::{Indexable, Rotate, Transform, Norm};
use na;
use implicit::{Implicit, PreferedSamplingDirections};
use geom::Cone;
use math::{Point, Vect};

impl<_M: Transform<Point> + Rotate<Vect>>
Implicit<Point, Vect, _M> for Cone {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Point {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        vres[1] = na::zero();

        if vres.normalize().is_zero() {
            vres = na::zero();

            if local_dir.at(1).is_negative() {
                vres[1] = -self.half_height()
            }
            else {
                vres[1] = self.half_height()
            }
        }
        else {
            vres = vres * self.radius();
            vres[1] = -self.half_height();

            if na::dot(&local_dir, &vres) < local_dir.at(1) * self.half_height() {
                vres = na::zero();
                vres[1] = self.half_height()
            }
        }

        m.transform(vres.as_pnt())
    }
}

impl<_M: Rotate<Vect>>
PreferedSamplingDirections<Vect, _M> for Cone {
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
