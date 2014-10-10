use na::{Transform, Rotate};
use implicit::{Implicit, PreferedSamplingDirections};
use implicit;
use geom::Convex;
use math::{Point, Vect};

impl<M: Transform<Point> + Rotate<Vect>> Implicit<Point, Vect, M> for Convex {
    #[inline]
    fn support_point(&self, m: &M, dir: &Vect) -> Point {
        let local_dir = m.inv_rotate(dir);

        let best_pt = implicit::point_cloud_support_point(&local_dir, self.pts());

        m.transform(&best_pt)
    }
}

impl<M> PreferedSamplingDirections<Vect, M> for Convex {
    #[inline(always)]
    fn sample(&self, _: &M, _: |Vect| -> ()) {
    }
}
