
use std::num::Signed;
use nalgebra::na::{Indexable, Transform, Rotate};
use nalgebra::na;
use geom::Box;
use implicit::{HasMargin, Implicit, PreferedSamplingDirections};
use math::{N, V};

impl HasMargin for Box {
    #[inline]
    fn margin(&self) -> N {
        self.margin()
    }
}

impl<_M: Rotate<V> + Transform<V>>
Implicit<V, _M> for Box {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres: V = na::zero();

        let he = self.half_extents();
        for i in range(0u, na::dim::<V>()) {
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

impl<_M: Rotate<V>>
PreferedSamplingDirections<V, _M> for Box {
    #[inline(always)]
    fn sample(&self, transform: &_M, f: |V| -> ()) {
        na::canonical_basis(|e: V| {
            let re = transform.rotate(&e);
            f(-re);
            f(re);
            true
        })
    }
}
