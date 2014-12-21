use na::{Zero, One};
use na;
use utils::symbolic::{UnivariateFn, BivariateFn, SymAdd, SymMult, SymSub, SymNeg};
use utils::symbolic;

/// A bivariate function of `V`.
#[deriving(Clone)]
pub struct V;

/// A bivariate function of `v`.
#[inline]
pub fn v() -> V {
    V
}

impl<N: Zero + One + Copy + Clone> BivariateFn<N, N> for V {
    #[inline]
    fn d0(&self, _: N, v: N) -> N {
        v
    }

    #[inline]
    fn du(&self, _: N, _: N) -> N {
        na::zero()
    }

    #[inline]
    fn dv(&self, _: N, _: N) -> N {
        na::one()
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
        if n == 0 {
            if k == 0 {
                return self.d0(u, v);
            }
            else if k == 1 {
                return self.dv(u, v);
            }
        }

        na::zero()
    }
}

impl_ops_noparam!(V);
