
use std::num::Signed;
use nalgebra::na::{Indexable, AlgebraicVecExt, Transform, Rotate};
use nalgebra::na;
use geom::Box;
use implicit::{HasMargin, Implicit};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;

impl<N: Clone, V: Clone> HasMargin<N> for Box<N, V> {
    #[inline]
    fn margin(&self) -> N {
        self.margin()
    }
}

impl<N: Algebraic + Signed + Clone,
     V: Clone + AlgebraicVecExt<N>,
     M: Rotate<V> + Transform<V>>
Implicit<N, V, M> for Box<N, V> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres: V = na::zero();

        let he = self.half_extents();
        for i in range(0u, na::dim::<V>()) {
            if local_dir.at(i).is_negative() {
                vres.set(i, -he.at(i));
            }
            else {
                vres.set(i, he.at(i));
            }
        }

        m.transform(&vres)
    }
}

impl<N,
     V: AlgebraicVecExt<N>,
     M: Rotate<V>>
PreferedSamplingDirections<V, M> for Box<N, V> {
    #[inline(always)]
    fn sample(&self, transform: &M, f: |V| -> ()) {
        na::canonical_basis(|e: V| {
            let re = transform.rotate(&e);
            f(-re);
            f(re);
            true
        })
    }
}
