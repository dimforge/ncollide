use utils::symbolic::{UnivariateFn, BivariateFn, SymSub, SymMult, SymNeg};
use utils::symbolic;

/// The addition operator.
#[deriving(Clone, Show, Copy)]
pub struct SymAdd<A, B> {
    a: A,
    b: B
}

/// Symbolic representation of the addition of two functions.
#[inline]
pub fn add<A, B>(a: A, b: B) -> SymAdd<A, B> {
    SymAdd { a: a, b: b }
}

impl<A: UnivariateFn<N, O>, B: UnivariateFn<N, O>, N: Copy, O: Add<O, O>>
UnivariateFn<N, O> for SymAdd<A, B> {
    #[inline]
    fn d0(&self, t: N) -> O {
        self.a.d0(t) + self.b.d0(t)
    }

    #[inline]
    fn d1(&self, t: N) -> O {
        self.a.d1(t) + self.b.d1(t)
    }

    #[inline]
    fn d2(&self, t: N) -> O {
        self.a.d2(t) + self.b.d2(t)
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> O {
        self.a.dn(t, n) + self.b.dn(t, n)
    }
}

impl<A: BivariateFn<N, O>, B: BivariateFn<N, O>, N: Copy, O: Add<O, O> + Clone>
BivariateFn<N, O> for SymAdd<A, B> {
    #[inline]
    fn d0(&self, u: N, v: N) -> O {
        self.a.d0(u, v) + self.b.d0(u, v)
    }

    #[inline]
    fn du(&self, u: N, v: N) -> O {
        self.a.du(u, v) + self.b.du(u, v)
    }

    #[inline]
    fn dv(&self, u: N, v: N) -> O {
        self.a.dv(u, v) + self.b.dv(u, v)
    }

    #[inline]
    fn duu(&self, u: N, v: N) -> O {
        self.a.duu(u, v) + self.b.duu(u, v)
    }

    #[inline]
    fn dvv(&self, u: N, v: N) -> O {
        self.a.dvv(u, v) + self.b.dvv(u, v)
    }

    #[inline]
    fn duv(&self, u: N, v: N) -> O {
        self.a.duv(u, v) + self.b.duv(u, v)
    }

    #[inline]
    fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> O {
        self.a.duv_nk(u, v, n, k) + self.b.duv_nk(u, v, n, k)
    }
}

impl_ops_bin!(SymAdd);
