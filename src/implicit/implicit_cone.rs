use std::num::{Zero, One};
use nalgebra::na::{Indexable, VecExt, AlgebraicVecExt, Rotate, Transform};
use nalgebra::na;
use implicit::{Implicit, HasMargin};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;
use geom::Cone;

impl<N: Clone> HasMargin<N> for Cone<N> {
    #[inline]
    fn margin(&self) -> N {
        self.margin()
    }
}

impl<N: Clone + Signed + Algebraic + Ord,
     V: Clone + AlgebraicVecExt<N>,
     M: Transform<V> + Rotate<V>>
Implicit<N, V, M> for Cone<N> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        vres.set(0, na::zero());

        if vres.normalize().is_zero() {
            vres = na::zero();

            if local_dir.at(0).is_negative() {
                vres.set(0, -self.half_height())
            }
            else {
                vres.set(0, self.half_height())
            }
        }
        else {
            vres = vres * self.radius();
            vres.set(0, -self.half_height());

            if na::dot(&local_dir, &vres) < local_dir.at(0) * self.half_height() {
                vres = na::zero();
                vres.set(0, self.half_height())
            }
        }

        m.transform(&vres)
    }
}

impl<N: One,
     V: VecExt<N>,
     M: Rotate<V>>
PreferedSamplingDirections<V, M> for Cone<N> {
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
