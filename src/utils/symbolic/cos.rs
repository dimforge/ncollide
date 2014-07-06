use nalgebra::na::Cast;
use utils;
use utils::symbolic::{UnivariateFn, SymAdd, SymMult, SymSub, SymNeg, SymComp};
use utils::symbolic;

/// The cosinus function.
#[deriving(Clone)]
pub struct Cos;

/// The cosinus function.
#[inline]
pub fn cos<A>(a: A) -> SymComp<Cos, A> {
    symbolic::comp(Cos, a)
}

impl<N: FloatMath + Cast<f64>> UnivariateFn<N, N> for Cos {
    #[inline]
    fn d0(&self, t: N) -> N {
        t.cos()
    }

    #[inline]
    fn d1(&self, t: N) -> N {
        -t.sin()
    }

    #[inline]
    fn d2(&self, t: N) -> N {
        -t.cos()
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> N {
        utils::dcos(n, t)
    }
}

impl_ops_noparam!(Cos)
