use nalgebra::na::{Transform, Rotate};
use nalgebra::na;
use implicit::{Implicit, HasMargin, PreferedSamplingDirections};
use geom::Segment;
use math::{Scalar, Vect};

impl HasMargin for Segment {
    #[inline]
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

impl<_M: Transform<Vect> + Rotate<Vect>>
Implicit<Vect, _M> for Segment {
    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vect) -> Vect {
        let local_dir = m.inv_rotate(dir);

        if na::dot(self.a(), &local_dir) > na::dot(self.b(), &local_dir) {
            m.transform(self.a())
        }
        else {
            m.transform(self.b())
        }
    }
}

impl<_V, _M> PreferedSamplingDirections<_V, _M> for Segment {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |_V| -> ()) {
    }
}
