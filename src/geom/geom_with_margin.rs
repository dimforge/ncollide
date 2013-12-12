#[doc(hidden)];

use std::num::Zero;
use nalgebra::na::AlgebraicVec;
use geom::{Implicit, HasMargin};

// This extends the wrapped geometry with its margin. I.e. the `support_point_withou_margin` method
// will return the wrapped geometry `support_point` instead.
pub struct GeomWithMargin<'a, G> {
    priv geom: &'a G
}

impl<'a, G> GeomWithMargin<'a, G> {
    pub fn new(geom: &'a G) -> GeomWithMargin<'a, G> {
        GeomWithMargin {
            geom: geom
        }
    }
}

impl<'a, N: Zero, G> HasMargin<N> for GeomWithMargin<'a, G> {
    #[inline]
    fn margin(&self) -> N {
        Zero::zero()
    }
}


impl<'a,
     N: Algebraic + Zero,
     V: AlgebraicVec<N>,
     M,
     G: Implicit<N, V, M>>
Implicit<N, V, M> for GeomWithMargin<'a, G> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> V {
        self.geom.support_point(m, dir)
    }

    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        self.geom.support_point(m, dir)
    }
}
