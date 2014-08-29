use nalgebra::na::{DMat, Indexable};

/// Trait implemented by smooth bivariate functions.
pub trait BivariateFn<N: Copy, O: Clone> {
    /// Evaluates the function.
    fn d0(&self, u: N, v: N) -> O;

    /// Same as `d0`.
    #[inline]
    fn beval(&self, u: N, v: N) -> O {
        self.d0(u, v)
    }

    /// Evaluates the first derivative wrt. `u`.
    fn du(&self, u: N, v: N) -> O;

    /// Evaluates the firt derivative wrt. `v`.
    fn dv(&self, u: N, v: N) -> O;

    /// Evaluates the second derivative wrt. `u`.
    fn duu(&self, u: N, v: N) -> O;

    /// Evaluates the second derivative wrt. `v`.
    fn dvv(&self, u: N, v: N) -> O;

    /// Evaluates the second derivative wrt. `u` and `v`.
    fn duv(&self, u: N, v: N) -> O;

    /// Evaluate the n-th derivative wrt. `u` and k-th derivative wrt. `v`.
    fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> O;

    /// Evaluate all the n-th derivative wrt. `u` and k-th derivative wrt. `v`.
    #[inline]
    fn duv_nk_all(&self, u: N, v: N, n: uint, k: uint, out: &mut DMat<O>) {
        assert!(out.nrows() == n && out.ncols() == k);

        for i in range(0, n) {
            for j in range(0, n) {
                out.set((i, j), self.duv_nk(u, v, i, j))
            }
        }
    }
}

impl<'a, N: Copy, O: Clone, T: BivariateFn<N, O>> BivariateFn<N, O> for &'a T {
    #[inline]
    fn d0(&self, u: N, v: N) -> O {
        (*self).d0(u, v)
    }

    #[inline]
    fn beval(&self, u: N, v: N) -> O {
        (*self).beval(u, v)
    }

    #[inline]
    fn du(&self, u: N, v: N) -> O {
        (*self).du(u, v)
    }

    #[inline]
    fn dv(&self, u: N, v: N) -> O {
        (*self).dv(u, v)
    }

    #[inline]
    fn duu(&self, u: N, v: N) -> O {
        (*self).duu(u, v)
    }

    #[inline]
    fn dvv(&self, u: N, v: N) -> O {
        (*self).dvv(u, v)
    }

    #[inline]
    fn duv(&self, u: N, v: N) -> O {
        (*self).duv(u, v)
    }

    #[inline]
    fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> O {
        (*self).duv_nk(u, v, n, k)
    }

    #[inline]
    fn duv_nk_all(&self, u: N, v: N, n: uint, k: uint, out: &mut DMat<O>) {
        (*self).duv_nk_all(u, v, n, k, out)
    }
}

impl<'a, N: Copy, O: Clone> BivariateFn<N, O> for &'a BivariateFn<N, O> + 'a {
    #[inline]
    fn d0(&self, u: N, v: N) -> O {
        (*self).d0(u, v)
    }

    #[inline]
    fn beval(&self, u: N, v: N) -> O {
        (*self).beval(u, v)
    }

    #[inline]
    fn du(&self, u: N, v: N) -> O {
        (*self).du(u, v)
    }

    #[inline]
    fn dv(&self, u: N, v: N) -> O {
        (*self).dv(u, v)
    }

    #[inline]
    fn duu(&self, u: N, v: N) -> O {
        (*self).duu(u, v)
    }

    #[inline]
    fn dvv(&self, u: N, v: N) -> O {
        (*self).dvv(u, v)
    }

    #[inline]
    fn duv(&self, u: N, v: N) -> O {
        (*self).duv(u, v)
    }

    #[inline]
    fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> O {
        (*self).duv_nk(u, v, n, k)
    }

    #[inline]
    fn duv_nk_all(&self, u: N, v: N, n: uint, k: uint, out: &mut DMat<O>) {
        (*self).duv_nk_all(u, v, n, k, out)
    }
}
