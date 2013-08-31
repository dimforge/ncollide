//!
//! Support mapping based reflected geometry.
//!

use nalgebra::vec::AlgebraicVec;
use geom::implicit::{Implicit, HasMargin};

/**
 * Implicit represention of the reflection of a geometric object.
 * A reflection is obtained with the central symetry wrt the origin.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct Reflection<'self, G> {
    priv g: &'self G
}

impl<'self, G> Reflection<'self, G> {
    /// Build the reflection of a geometry. Since the representation is implicit,
    /// the reflection computation is done in constant time.
    #[inline]
    pub fn new(g: &'self G) -> Reflection<'self, G> {
        Reflection { g: g }
    }
}

impl<'self, N, G: HasMargin<N>> HasMargin<N> for Reflection<'self, G> {
    #[inline]
    fn margin(&self) -> N {
        self.g.margin()
    }
}

impl<'self, N: Algebraic, V: AlgebraicVec<N>, M, G: Implicit<N, V, M>> Implicit<N, V, M> for Reflection<'self, G> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> V {
        -self.g.support_point(m, &-dir)
    }

    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        -self.g.support_point_without_margin(m, &-dir)
    }
}
