use nalgebra::na::{AlgebraicVec, Transform, Rotate};
use nalgebra::na;
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;
use implicit::{Implicit, HasMargin};
use geom::Triangle;

impl<N: Clone, V> HasMargin<N> for Triangle<N, V> {
    #[inline]
    fn margin(&self) -> N {
        self.margin()
    }
}

impl<N: Algebraic + Clone + Ord,
     V: AlgebraicVec<N>,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for Triangle<N, V> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let d1 = na::dot(self.a(), &local_dir);
        let d2 = na::dot(self.b(), &local_dir);
        let d3 = na::dot(self.c(), &local_dir);

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

impl<N, V, M> PreferedSamplingDirections<V, M> for Triangle<N, V> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
