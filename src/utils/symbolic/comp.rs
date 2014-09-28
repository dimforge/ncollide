use std::num::Zero;
use na::Cast;
use utils::symbolic::{UnivariateFn, BivariateFn, SymAdd, SymSub, SymMult, SymNeg};
use utils::symbolic;

/// The composition operator.
#[deriving(Clone, Show)]
pub struct SymComp<A, B> {
    a: A,
    b: B
}

/// Symbolic representation of the composition of two functions.
#[inline]
pub fn comp<A, B>(a: A, b: B) -> SymComp<A, B> {
    SymComp { a: a, b: b }
}

// Univariate o Univariate => Univariate
impl<A: UnivariateFn<O, O>, B: UnivariateFn<N, O>, N: Copy, O: Copy + Mul<O, O> + Add<O, O> + Zero + Cast<f64>>
UnivariateFn<N, O> for SymComp<A, B> {
    #[inline]
    fn d0(&self, t: N) -> O {
        self.a.d0(self.b.d0(t))
    }

    #[inline]
    fn d1(&self, t: N) -> O {
        self.b.d1(t) * self.a.d1(self.b.d0(t))
    }

    #[inline]
    fn d2(&self, t: N) -> O {
        let b1 = self.b.d1(t);

        self.b.d2(t) * self.a.d1(self.b.d0(t)) +
        b1             * b1 * self.a.d2(self.b.d0(t))
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> O {
        // FIXME: dont use a recursive definition.
        // However, the closed-form formula is complicated.
        //
        // NOTE: We need recursive polymorphism.
        // Therefore, we have to box things that goes on a `mult` to avoid an infinitely recursive
        // monomorphization.
        if n == 0 {
            self.ueval(t)
        }
        else {
            let db = symbolic::deriv(&self.b);
            let da = &symbolic::deriv(&self.a) as &UnivariateFn<O, O>;
            let b  = &self.b as &UnivariateFn<N, O>;

            symbolic::mult(db, comp(da, b)).dn(t, n - 1)
        }
    }
}

// Univariate o Bivariate => Bivariate
impl<A: UnivariateFn<O, O>, B: BivariateFn<N, O>, N: Copy, O: Copy + Mul<O, O> + Add<O, O> + Clone>
BivariateFn<N, O> for SymComp<A, B> {
    #[inline]
    fn d0(&self, u: N, v: N) -> O {
        self.a.d0(self.b.d0(u, v))
    }

    #[inline]
    fn du(&self, u: N, v: N) -> O {
        self.b.du(u, v) * self.a.d1(self.b.d0(u, v))
    }

    #[inline]
    fn dv(&self, u: N, v: N) -> O {
        self.b.dv(u, v) * self.a.d1(self.b.d0(u, v))
    }

    #[inline]
    fn duu(&self, u: N, v: N) -> O {
        let b0 = self.b.d0(u, v);
        let bu = self.b.du(u, v);

        self.b.duu(u, v) * self.a.d1(b0) +
        bu * bu * self.a.d2(b0)
    }

    #[inline]
    fn dvv(&self, u: N, v: N) -> O {
        let b0 = self.b.d0(u, v);
        let bv = self.b.dv(u, v);

        self.b.dvv(u, v) * self.a.d1(b0) +
        bv * bv * self.a.d2(b0)
    }

    #[inline]
    fn duv(&self, u: N, v: N) -> O {
        let b0 = self.b.d0(u, v);

        self.b.duv(u, v) * self.a.d1(b0) +
        self.b.du(u, v)  * self.b.dv(u, v) * self.a.d2(b0)
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
                // differenciate wrt v
                let bv = symbolic::deriv_v(&self.b);
                let da = &symbolic::deriv(&self.a) as &UnivariateFn<O, O>;
                let b  = &self.b as &BivariateFn<N, O>;

                symbolic::mult(bv, comp(da, b)).duv_nk(u, v, n, k - 1)
            }
        }
        else {
            // diefferenciate wrt u
            let bu = symbolic::deriv_u(&self.b);
            let da = &symbolic::deriv(&self.a) as &UnivariateFn<O, O>;
            let b  = &self.b as &BivariateFn<N, O>;

            symbolic::mult(bu, comp(da, b)).duv_nk(u, v, n - 1, k)
        }
    }
}

impl_ops_bin!(SymComp)
