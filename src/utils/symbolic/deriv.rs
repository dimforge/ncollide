use utils::symbolic::{UnivariateFn, BivariateFn, SymAdd, SymSub, SymMult, SymNeg};
use utils::symbolic;

/// The derivative of `A`.
#[derive(Clone, Copy)]
pub struct Deriv<A> {
    a: A
}

/// The derivative of `A`.
#[inline]
pub fn deriv<A>(a: A) -> Deriv<A> {
    Deriv {
        a: a
    }
}

impl<A: UnivariateFn<N, O>, N: Copy, O> UnivariateFn<N, O> for Deriv<A> {
    #[inline]
    fn d0(&self, t: N) -> O {
        self.a.d1(t)
    }

    #[inline]
    fn d1(&self, t: N) -> O {
        self.a.d2(t)
    }

    #[inline]
    fn d2(&self, t: N) -> O {
        self.a.dn(t, 3)
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> O {
        self.a.dn(t, n + 1)
    }
}

impl_ops!(Deriv);

/// The derivative of `A` wrt. `u`.
#[derive(Clone)]
pub struct DerivU<A> {
    a: A
}

/// The derivative of `A` wrt. `u`.
#[inline]
pub fn deriv_u<A>(a: A) -> DerivU<A> {
    DerivU {
        a: a
    }
}

impl<A: BivariateFn<N, O>, N: Copy, O: Clone> BivariateFn<N, O> for DerivU<A> {
    #[inline]
    fn d0(&self, u: N, v: N) -> O {
        self.a.du(u, v)
    }

    #[inline]
    fn du(&self, u: N, v: N) -> O {
        self.a.duu(u, v)
    }

    #[inline]
    fn dv(&self, u: N, v: N) -> O {
        self.a.duv(u, v)
    }

    #[inline]
    fn duu(&self, u: N, v: N) -> O {
        self.a.duv_nk(u, v, 3, 0)
    }

    #[inline]
    fn dvv(&self, u: N, v: N) -> O {
        self.a.duv_nk(u, v, 1, 2)
    }

    #[inline]
    fn duv(&self, u: N, v: N) -> O {
        self.a.duv_nk(u, v, 2, 1)
    }

    #[inline]
    fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> O {
        self.a.duv_nk(u, v, n + 1, k)
    }
}

impl_ops!(DerivU);

/// The derivative of `A` wrt. `v`.
#[derive(Clone)]
pub struct DerivV<A> {
    a: A
}

/// The derivative of `A` wrt. `v`.
#[inline]
pub fn deriv_v<A>(a: A) -> DerivV<A> {
    DerivV {
        a: a
    }
}

impl<A: BivariateFn<N, O>, N: Copy, O: Clone> BivariateFn<N, O> for DerivV<A> {
    #[inline]
    fn d0(&self, u: N, v: N) -> O {
        self.a.dv(u, v)
    }

    #[inline]
    fn du(&self, u: N, v: N) -> O {
        self.a.duv(u, v)
    }

    #[inline]
    fn dv(&self, u: N, v: N) -> O {
        self.a.dvv(u, v)
    }

    #[inline]
    fn duu(&self, u: N, v: N) -> O {
        self.a.duv_nk(u, v, 2, 1)
    }

    #[inline]
    fn dvv(&self, u: N, v: N) -> O {
        self.a.duv_nk(u, v, 3, 0)
    }

    #[inline]
    fn duv(&self, u: N, v: N) -> O {
        self.a.duv_nk(u, v, 1, 2)
    }

    #[inline]
    fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> O {
        self.a.duv_nk(u, v, n, k + 1)
    }
}

impl_ops!(DerivV);
