use std::num::Bounded;
use nalgebra::na::{Transform, Rotate};
use nalgebra::na;
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Convex;
use math::{Scalar, Vector};

impl HasMargin for Convex {
    #[inline]
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

impl<_M: Transform<Vector> + Rotate<Vector>>
Implicit<Vector, _M> for Convex {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vector) -> Vector {
        let local_dir = m.inv_rotate(dir);

        let _max: Scalar = Bounded::max_value();
        let mut best_dot = -_max;
        let mut best_pt  = &self.pts()[0];

        for p in self.pts().iter() {
            let dot = na::dot(p, &local_dir);

            if dot > best_dot {
                best_dot = dot;
                best_pt  = p;
            }
        }


        m.transform(best_pt)
    }
}

impl<_M>
PreferedSamplingDirections<Vector, _M> for Convex {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |Vector| -> ()) {
    }
}
