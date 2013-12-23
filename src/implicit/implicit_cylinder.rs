use std::num::{Zero, One};
use nalgebra::na::{Indexable, VecExt, AlgebraicVecExt, Rotate, Transform};
use nalgebra::na;
use implicit::{Implicit, HasMargin};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;
use geom::Cylinder;


impl<N: Clone> HasMargin<N> for Cylinder<N> {
    #[inline]
    fn margin(&self) -> N {
        self.margin()
    }
}

impl<N: Clone + Algebraic + Signed,
     V: Clone + AlgebraicVecExt<N>,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for Cylinder<N> {
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir.at(0).is_negative();

        vres.set(0, Zero::zero());

        if vres.normalize().is_zero() {
            vres = Zero::zero()
        }
        else {
            vres = vres * self.radius();
        }

        if negative {
            vres.set(0, -self.half_height())
        }
        else {
            vres.set(0, self.half_height())
        }

        m.transform(&vres)
    }
}

impl<N: One,
     V: VecExt<N>,
     M: Rotate<V>>
PreferedSamplingDirections<V, M> for Cylinder<N> {
    #[inline(always)]
    fn sample(&self, transform: &M, f: |V| -> ()) {
        // Sample along the principal axis
        let mut v: V = na::zero();
        v.set(0, na::one());

        let rv = transform.rotate(&v);
        f(-rv);
        f(rv);
    }
}
