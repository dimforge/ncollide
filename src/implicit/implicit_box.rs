
use std::num::Signed;
use nalgebra::na::{Indexable, Transform, Rotate};
use nalgebra::na;
use geom::Cuboid;
use implicit::{HasMargin, Implicit, PreferedSamplingDirections};
use math::{Scalar, Vect};

impl HasMargin for Cuboid {
    #[inline]
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

impl<_M: Rotate<Vect> + Transform<Vect>>
Implicit<Vect, _M> for Cuboid {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vect) -> Vect {
        let local_dir = m.inv_rotate(dir);

        let mut vres: Vect = na::zero();

        let he = self.half_extents();
        for i in range(0u, na::dim::<Vect>()) {
            if local_dir.at(i).is_negative() {
                vres.set(i, -he.at(i));
            }
            else {
                vres.set(i, he.at(i));
            }
        }

        m.transform(&vres)
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
