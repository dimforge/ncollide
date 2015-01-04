use std::ops::{Add, Sub, Mul, Neg};
use na::{Cast, Zero};
use na;
use symbolic::{UnivariateFn, BivariateFn, SymAdd, SymSub, SymNeg};
use symbolic;

/// The multiplication operator.
#[derive(Clone, Show, Copy)]
pub struct SymMult<A, B> {
    a: A,
    b: B
}

/// Symbolic representation of the multiplication of two functions.
#[inline]
pub fn mult<A, B>(a: A, b: B) -> SymMult<A, B> {
    SymMult { a: a, b: b }
}

impl<A: UnivariateFn<N, O>, B: UnivariateFn<N, O>, N: Copy, O: Mul<O, O> + Add<O, O> + Zero + Cast<f64> + Copy>
UnivariateFn<N, O> for SymMult<A, B> {
    #[inline]
    fn d0(&self, t: N) -> O {
        self.a.d0(t) * self.b.d0(t)
    }

    #[inline]
    fn d1(&self, t: N) -> O {
        self.a.d1(t) * self.b.d0(t) + self.a.d0(t) * self.b.d1(t)
    }

    #[inline]
    fn d2(&self, t: N) -> O {
        let a1b1 = self.a.d1(t) * self.b.d1(t);

        self.a.d2(t) * self.b.d0(t) + a1b1 +
        a1b1                        + self.a.d0(t) * self.b.d2(t)
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> O {
        let mut res = na::zero::<O>();

        for i in range(0, n + 1) {
            let coeff: O = na::cast(::binom(i, n) as f64);
            res = res + coeff * self.a.dn(t, i) * self.b.dn(t, n - i)
        }

        res
    }
}

impl<A: BivariateFn<N, O>, B: BivariateFn<N, O>, N: Copy, O: Mul<O, O> + Add<O, O> + Clone + Copy>
BivariateFn<N, O> for SymMult<A, B> {
    #[inline]
    fn d0(&self, u: N, v: N) -> O {
        self.a.d0(u, v) * self.b.d0(u, v)
    }

    #[inline]
    fn du(&self, u: N, v: N) -> O {
        self.a.du(u, v) * self.b.d0(u, v) +
        self.a.d0(u, v) * self.b.du(u, v)
    }

    #[inline]
    fn dv(&self, u: N, v: N) -> O {
        self.a.dv(u, v) * self.b.d0(u, v) +
        self.a.d0(u, v) * self.b.dv(u, v)
    }

    #[inline]
    fn duu(&self, u: N, v: N) -> O {
        let aubu = self.a.du(u, v) * self.b.du(u, v);

        self.a.duu(u, v) * self.b.d0(u, v) +
        aubu                                   +
        aubu                                   +
        self.a.d0(u, v) * self.b.duu(u, v)
    }

    #[inline]
    fn dvv(&self, u: N, v: N) -> O {
        let avbv = self.a.dv(u, v) * self.b.dv(u, v);

        self.a.dvv(u, v) * self.b.d0(u, v) +
        avbv                               +
        avbv                               +
        self.a.d0(u, v) * self.b.dvv(u, v)
    }

    #[inline]
    fn duv(&self, u: N, v: N) -> O {
        self.a.duv(u, v) * self.b.d0(u, v) +
        self.a.du(u, v)  * self.b.dv(u, v) +
        self.a.dv(u, v)  * self.b.du(u, v) +
        self.a.d0(u, v)  * self.b.duv(u, v)
    }

    #[inline]
    fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> O {
        // FIXME: dont use a recursive definition.
        // However, the closed-form formula is complicated.
        //
        // NOTE: We need recursive polymorphism.
        // Therefore, we have to box things that goes on a `mult` to avoid an infinitely recursive
        // monomorphization.
        if n == 0 {
            if k == 0 {
                self.beval(u, v)
            }
            else {
                // differenciate wrt. v
                let av   = &symbolic::deriv_v(&self.a) as &BivariateFn<N, O>;
                let bv   = &symbolic::deriv_v(&self.b) as &BivariateFn<N, O>;
                let b0   = &self.b as &BivariateFn<N, O>;
                let a0   = &self.a as &BivariateFn<N, O>;

                symbolic::add(mult(av, b0), mult(a0, bv)).duv_nk(u, v, n, k - 1)
            }
        }
        else {
            // differenciate wrt. u
            let au   = &symbolic::deriv_u(&self.a) as &BivariateFn<N, O>;
            let bu   = &symbolic::deriv_u(&self.b) as &BivariateFn<N, O>;
            let b0   = &self.b as &BivariateFn<N, O>;
            let a0   = &self.a as &BivariateFn<N, O>;

            symbolic::add(mult(au, b0), mult(a0, bu)).duv_nk(u, v, n - 1, k)
        }
    }
}
impl_ops_bin!(SymMult);
