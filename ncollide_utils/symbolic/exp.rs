use std::ops::{Add, Sub, Mul, Neg};
use symbolic::{UnivariateFn, SymAdd, SymMult, SymSub, SymNeg, SymComp};
use symbolic;
use math::Scalar;

/// The exponential function.
#[derive(Clone, Copy)]
pub struct Exp;

/// The exponential function.
#[inline]
pub fn exp<A>(a: A) -> SymComp<Exp, A> {
    symbolic::comp(Exp, a)
}

impl<N: Scalar> UnivariateFn<N, N> for Exp {
    #[inline]
    fn d0(&self, t: N) -> N {
        t.exp()
    }

    #[inline]
    fn d1(&self, t: N) -> N {
        t.exp()
    }

    #[inline]
    fn d2(&self, t: N) -> N {
        t.exp()
    }

    #[inline]
    fn dn(&self, t: N, _: uint) -> N {
        t.exp()
    }
}

impl_ops_noparam!(Exp);
