use na;
use na::{Indexable, Rotate, Transform};
use implicit::{Implicit, PreferedSamplingDirections};
use geom::Capsule;
use math::{Point, Vect};

impl<_M: Transform<Point> + Rotate<Vect>>
Implicit<Point, Vect, _M> for Capsule {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Point {
        let local_dir = m.inv_rotate(dir);

        let mut pres: Point = na::orig();

        if local_dir.at(1).is_negative() {
            pres[1] = -self.half_height()
        }
        else {
            pres[1] = self.half_height()
        }

        m.transform(&(pres + na::normalize(&local_dir) * self.radius()))
    }
}

impl<Vect, _M>
PreferedSamplingDirections<Vect, _M> for Capsule {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |Vect| -> ()) {
    }
}
