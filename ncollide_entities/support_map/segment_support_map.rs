use na::{Transform, Rotate};
use na;
use support_map::{SupportMap, PreferedSamplingDirections};
use shape::Segment;
use math::{Scalar, Point, Vect};


impl<N, P, V, M> SupportMap<P, V, M> for Segment<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        let local_dir = m.inv_rotate(dir);

        if na::dot(self.a().as_vec(), &local_dir) > na::dot(self.b().as_vec(), &local_dir) {
            m.transform(self.a())
        }
        else {
            m.transform(self.b())
        }
    }
}

impl<P, V, M> PreferedSamplingDirections<V, M> for Segment<P> {
    #[inline(always)]
    fn sample(&self, _: &M, _: &mut FnMut(V)) {
    }
}
