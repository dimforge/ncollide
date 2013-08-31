#[doc(hidden)];

use std::num::Zero;
use nalgebra::vec::AlgebraicVec;
use geom::implicit::{Implicit, HasMargin};

// This extends the wrapped geometry with its margin. I.e. the `support_point_withou_margin` method
// will return the wrapped geometry `support_point` instead.
pub struct GeomWithMargin<'self, G> {
    priv geom: &'self G
}

impl<'self, G> GeomWithMargin<'self, G> {
    pub fn new(geom: &'self G) -> GeomWithMargin<'self, G> {
        GeomWithMargin {
            geom: geom
        }
    }
}

impl<'self, N: Zero, G> HasMargin<N> for GeomWithMargin<'self, G> {
    #[inline]
    fn margin(&self) -> N {
        Zero::zero()
    }
}


impl<'self,
     N: Algebraic + Zero,
     V: AlgebraicVec<N>,
     M,
     G: Implicit<N, V, M>>
Implicit<N, V, M> for GeomWithMargin<'self, G> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> V {
        self.geom.support_point(m, dir)
    }

    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        self.geom.support_point(m, dir)
    }
}
