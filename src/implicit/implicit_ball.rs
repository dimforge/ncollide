use na;
use na::Translation;
use implicit::{PreferedSamplingDirections, Implicit};
use geom::Ball;
use math::{Point, Vect};

impl<_M: Translation<Vect>> Implicit<Point, Vect, _M> for Ball {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Point {
        m.translation().as_pnt() + na::normalize(dir) * self.radius()
    }
}

impl<_M> PreferedSamplingDirections<Vect, _M> for Ball {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |Vect| -> ()) {
    }
}
