#[doc(hidden)];

use std::num::Zero;
use implicit::{Implicit, HasMargin};
use math::{N, V};

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

impl<'a, G> HasMargin for GeomWithMargin<'a, G> {
    #[inline]
    fn margin(&self) -> N {
        Zero::zero()
    }
}


impl<'a, _M, G: Implicit<V, _M>>
Implicit<V, _M> for GeomWithMargin<'a, G> {
    #[inline]
    fn support_point(&self, m: &_M, dir: &V) -> V {
        self.geom.support_point(m, dir)
    }

    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &V) -> V {
        self.geom.support_point(m, dir)
    }
}
