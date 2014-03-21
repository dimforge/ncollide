//! A geometry enlarged with its margin.

use std::num::Zero;
use implicit::{Implicit, HasMargin};
use math::{Scalar, Vect};

/// Extends the wrapped geometry with its margin.
///
/// The purpose of this structure as to make the `support_point_without_margin` method return
/// result of the wrapped geometry `support_point` method instead.
pub struct GeomWithMargin<'a, G> {
    priv geom: &'a G
}

impl<'a, G> GeomWithMargin<'a, G> {
    /// Creates a new `GeomWithMargin`.
    pub fn new(geom: &'a G) -> GeomWithMargin<'a, G> {
        GeomWithMargin {
            geom: geom
        }
    }
}

impl<'a, G> HasMargin for GeomWithMargin<'a, G> {
    #[inline]
    fn margin(&self) -> Scalar {
        Zero::zero()
    }
}


impl<'a, _M, G: Implicit<Vect, _M>>
Implicit<Vect, _M> for GeomWithMargin<'a, G> {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Vect {
        self.geom.support_point(m, dir)
    }

    #[inline]
    fn support_point_without_margin(&self, m: &_M, dir: &Vect) -> Vect {
        self.geom.support_point(m, dir)
    }
}
