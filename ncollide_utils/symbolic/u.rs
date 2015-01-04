use std::ops::{Add, Sub, Mul, Neg};
use na::{Zero, One};
use na;
use symbolic::{UnivariateFn, BivariateFn, SymAdd, SymMult, SymSub, SymNeg};
use symbolic;

/// A bivariate function of `U`.
#[derive(Clone, Copy)]
pub struct U;

/// A bivariate function of `u`.
#[inline]
pub fn u() -> U {
    U
}

impl<N: Zero + One + Copy + Clone> BivariateFn<N, N> for U {
    #[inline]
    fn d0(&self, u: N, _: N) -> N {
        u
    }

    #[inline]
    fn du(&self, _: N, _: N) -> N {
        na::one()
    }

    #[inline]
    fn dv(&self, _: N, _: N) -> N {
        na::zero()
    }

    #[inline]
    fn duu(&self, _: N, _: N) -> N {
        na::zero()
    }

    #[inline]
    fn dvv(&self, _: N, _: N) -> N {
        na::zero()
    }

    #[inline]
    fn duv(&self, _: N, _: N) -> N {
        na::zero()
    }

    #[inline]
    fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> N {
        if k == 0 {
            if n == 0 {
                return self.d0(u, v);
            }
            else if n == 1 {
                return self.du(u, v);
            }
        }

        na::zero()
    }
}

impl_ops_noparam!(U);
