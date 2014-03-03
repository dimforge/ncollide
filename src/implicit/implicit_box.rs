
use std::num::Signed;
use nalgebra::na::{Indexable, Transform, Rotate};
use nalgebra::na;
use geom::Box;
use implicit::{HasMargin, Implicit, PreferedSamplingDirections};
use math::{Scalar, Vector};

impl HasMargin for Box {
    #[inline]
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

impl<_M: Rotate<Vector> + Transform<Vector>>
Implicit<Vector, _M> for Box {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vector) -> Vector {
        let local_dir = m.inv_rotate(dir);

        let mut vres: Vector = na::zero();

        let he = self.half_extents();
        for i in range(0u, na::dim::<Vector>()) {
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

impl<_M: Rotate<Vector>>
PreferedSamplingDirections<Vector, _M> for Box {
    #[inline(always)]
    fn sample(&self, transform: &_M, f: |Vector| -> ()) {
        na::canonical_basis(|e: Vector| {
            let re = transform.rotate(&e);
            f(-re);
            f(re);
            true
        })
    }
}
