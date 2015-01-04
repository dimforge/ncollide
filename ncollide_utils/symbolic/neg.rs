use std::ops::{Add, Sub, Mul, Neg};
use symbolic::{UnivariateFn, BivariateFn, SymSub, SymAdd, SymMult};
use symbolic;

/// The negation operator.
#[derive(Clone, Show, Copy)]
pub struct SymNeg<A> {
    a: A
}

/// Symbolic representation of the negation of a function.
#[inline]
pub fn neg<A>(a: A) -> SymNeg<A> {
    SymNeg { a: a }
}


impl<A: UnivariateFn<N, O>, N: Copy, O: Neg<O>>
UnivariateFn<N, O> for SymNeg<A> {
    #[inline]
    fn d0(&self, t: N) -> O {
        -self.a.ueval(t)
    }

    #[inline]
    fn d1(&self, t: N) -> O {
        -self.a.d1(t)
    }

    #[inline]
    fn d2(&self, t: N) -> O {
        -self.a.d2(t)
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> O {
        -self.a.dn(t, n)
    }
}

impl<A: BivariateFn<N, O>, N: Copy, O: Neg<O> + Clone> BivariateFn<N, O> for SymNeg<A> {
    #[inline]
    fn d0(&self, u: N, v: N) -> O {
        -self.a.beval(u, v)
    }

    #[inline]
    fn du(&self, u: N, v: N) -> O {
        -self.a.du(u, v)
    }

    #[inline]
    fn dv(&self, u: N, v: N) -> O {
        -self.a.dv(u, v)
    }

    #[inline]
    fn duu(&self, u: N, v: N) -> O {
        -self.a.duu(u, v)
    }

    #[inline]
    fn dvv(&self, u: N, v: N) -> O {
        -self.a.dvv(u, v)
    }

    #[inline]
    fn duv(&self, u: N, v: N) -> O {
        -self.a.duv(u, v)
    }

    #[inline]
    fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> O {
        -self.a.duv_nk(u, v, n, k)
    }
}

impl_ops!(SymNeg);
