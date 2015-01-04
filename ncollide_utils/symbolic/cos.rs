use std::ops::{Add, Sub, Mul, Neg};
use symbolic::{UnivariateFn, SymAdd, SymMult, SymSub, SymNeg, SymComp};
use symbolic;
use math::Scalar;

/// The cosinus function.
#[derive(Clone, Copy)]
pub struct Cos;

/// The cosinus function.
#[inline]
pub fn cos<A>(a: A) -> SymComp<Cos, A> {
    symbolic::comp(Cos, a)
}

impl<N: Scalar> UnivariateFn<N, N> for Cos {
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
        ::dcos(n, t)
    }
}

impl_ops_noparam!(Cos);
