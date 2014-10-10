
use std::num::Signed;
use na::{Indexable, Transform, Rotate};
use na;
use geom::Cuboid;
use implicit::{Implicit, PreferedSamplingDirections};
use math::{Vect, Point};

impl<_M: Rotate<Vect> + Transform<Point>>
Implicit<Point, Vect, _M> for Cuboid {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Point {
        let local_dir = m.inv_rotate(dir);

        let mut pres: Point = na::orig();

        let he = self.half_extents();
        for i in range(0u, na::dim::<Point>()) {
            if local_dir.at(i).is_negative() {
                pres[i] = -he[i];
            }
            else {
                pres[i] = he[i];
            }
        }

        m.transform(&pres)
    }
}

impl<_M: Rotate<Vect>>
PreferedSamplingDirections<Vect, _M> for Cuboid {
    #[inline(always)]
    fn sample(&self, transform: &_M, f: |Vect| -> ()) {
        na::canonical_basis(|e: Vect| {
            let re = transform.rotate(&e);
            f(-re);
            f(re);
            true
        })
    }
}
