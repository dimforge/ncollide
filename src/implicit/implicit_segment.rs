use na::{Transform, Rotate};
use na;
use implicit::{Implicit, PreferedSamplingDirections};
use geom::Segment;
use math::{Point, Vect};

impl<_M: Transform<Point> + Rotate<Vect>>
Implicit<Point, Vect, _M> for Segment {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Point {
        let local_dir = m.inv_rotate(dir);

        if na::dot(self.a().as_vec(), &local_dir) > na::dot(self.b().as_vec(), &local_dir) {
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
