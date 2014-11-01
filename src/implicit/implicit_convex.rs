use na::{Transform, Rotate};
use implicit::{Implicit, PreferedSamplingDirections};
use implicit;
use shape::Convex;
use math::{Scalar, Point, Vect};


impl<N, P, V, M> Implicit<P, V, M> for Convex<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        let local_dir = m.inv_rotate(dir);

        let best_pt = implicit::point_cloud_support_point(&local_dir, self.points());

        m.transform(&best_pt)
    }
}

impl<P, V, M> PreferedSamplingDirections<V, M> for Convex<P> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
