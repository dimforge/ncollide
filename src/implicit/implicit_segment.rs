use nalgebra::na::{AlgebraicVec, Transform, Rotate};
use nalgebra::na;
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;
use implicit::{Implicit, HasMargin};
use geom::Segment;

impl<N: Clone, V> HasMargin<N> for Segment<N, V> {
    #[inline]
    fn margin(&self) -> N {
        self.margin()
    }
}

impl<N: Algebraic + Clone + Ord,
     V: AlgebraicVec<N>,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for Segment<N, V> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        if na::dot(self.a(), &local_dir) > na::dot(self.b(), &local_dir) {
            m.transform(self.a())
        }
        else {
            m.transform(self.b())
        }
    }
}

impl<N, V, M> PreferedSamplingDirections<V, M> for Segment<N, V> {
    #[inline(always)]
    fn sample(&self, _: &M, _: |V| -> ()) {
    }
}
