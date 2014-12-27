use na::{Zero, One};
use na;
use utils::symbolic::{UnivariateFn, BivariateFn, SymAdd, SymMult, SymSub, SymNeg};
use utils::symbolic;

/// A univariate function of `T`.
#[deriving(Clone, Copy)]
pub struct T;

/// A univariate function of `t`.
#[inline]
pub fn t() -> T {
    T
}

impl<N: Zero + One + Copy> UnivariateFn<N, N> for T {
    #[inline]
    fn d0(&self, t: N) -> N {
        t
    }

    #[inline]
    fn d1(&self, _: N) -> N {
        na::one()
    }

    #[inline]
    fn d2(&self, _: N) -> N {
        na::zero()
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> N {
        if n == 0 {
            self.d0(t)
        }
        else if n == 1 {
            self.d1(t)
        }
        else {
            na::zero()
        }
    }
}

impl_ops_noparam!(T);
