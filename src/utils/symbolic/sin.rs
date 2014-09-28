use na::Cast;
use utils;
use utils::symbolic::{UnivariateFn, SymAdd, SymMult, SymSub, SymNeg, SymComp};
use utils::symbolic;

/// The sinus function.
#[deriving(Clone)]
pub struct Sin;

/// The sinus function.
#[inline]
pub fn sin<A>(a: A) -> SymComp<Sin, A> {
    symbolic::comp(Sin, a)
}

impl<N: FloatMath + Cast<f64>> UnivariateFn<N, N> for Sin {
    #[inline]
    fn d0(&self, t: N) -> N {
        t.sin()
    }

    #[inline]
    fn d1(&self, t: N) -> N {
        t.cos()
    }

    #[inline]
    fn d2(&self, t: N) -> N {
        -t.sin()
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> N {
        utils::dsin(n, t)
    }
}

impl_ops_noparam!(Sin)
