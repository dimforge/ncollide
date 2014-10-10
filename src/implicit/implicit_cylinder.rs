use std::num::Zero;
use na::{Indexable, Rotate, Transform, Norm};
use na;
use implicit::{Implicit, PreferedSamplingDirections};
use geom::Cylinder;
use math::{Point, Vect};


impl<_M: Transform<Point> + Rotate<Vect>>
Implicit<Point, Vect, _M> for Cylinder {
    fn support_point(&self, m: &_M, dir: &Vect) -> Point {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir.at(1).is_negative();

        vres[1]  = na::zero();

        if vres.normalize().is_zero() {
            vres = na::zero()
        }
        else {
            vres = vres * self.radius();
        }

        if negative {
            vres[1] = -self.half_height()
        }
        else {
            vres[1] = self.half_height()
        }

        m.transform(vres.as_pnt())
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
