use na::{Transform, Rotate};
use na;
use support_map::{SupportMap, PreferedSamplingDirections};
use shape::Triangle;
use math::{Scalar, Point, Vect};


impl<N, P, V, M> SupportMap<P, V, M> for Triangle<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        let local_dir = m.inv_rotate(dir);

        let d1 = na::dot(self.a().as_vec(), &local_dir);
        let d2 = na::dot(self.b().as_vec(), &local_dir);
        let d3 = na::dot(self.c().as_vec(), &local_dir);

        let res =
            if d1 > d2 {
                if d1 > d3 {
                    self.a()
                }
                else {
                    self.c()
                }
            }
            else {
                if d2 > d3 {
                    self.b()
                }
                else {
                    self.c()
                }
            };

        m.transform(res)
    }
}

impl<P, V, M> PreferedSamplingDirections<V, M> for Triangle<P> {
    #[inline(always)]
    fn sample(&self, _: &M, _: &mut FnMut(V)) {
    }
}
