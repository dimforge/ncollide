use nalgebra::na::{Transform, Rotate};
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use implicit;
use geom::Convex;
use math::{Scalar, Vect};

impl HasMargin for Convex {
    #[inline]
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

impl<_M: Transform<Vect> + Rotate<Vect>>
Implicit<Vect, _M> for Convex {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vect) -> Vect {
        let local_dir = m.inv_rotate(dir);

        let best_pt = implicit::point_cloud_support_point(&local_dir, self.pts());

        m.transform(&best_pt)
    }
}

impl<_M>
PreferedSamplingDirections<Vect, _M> for Convex {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |Vect| -> ()) {
    }
}
