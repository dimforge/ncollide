/// Trait implemented by smooth univariate functions.
pub trait UnivariateFn<N: Copy, O> {
    /// Evaluates the function.
    fn d0(&self, t: N) -> O;

    /// Same as `d0`.
    #[inline]
    fn ueval(&self, t: N) -> O {
        self.d0(t)
    }

    /// Evaluates the first derivative.
    fn d1(&self, t: N) -> O;

    /// Evaluates the second derivative.
    fn d2(&self, t: N) -> O;

    /// Evaluates the n-th derivative.
    fn dn(&self, t: N, n: uint) -> O;

    /// Evaluates the function and its first derivative.
    #[inline]
    fn d0_1(&self, t: N) -> (O, O) {
        (self.d0(t), self.d1(t))
    }

    /// Evaluates the function and its first two derivatives.
    #[inline]
    fn d0_1_2(&self, t: N) -> (O, O, O) {
        (self.d0(t), self.d1(t), self.d2(t))
    }

    /// Evaluates the function and all its derivative, up to the n-th (included).
    #[inline]
    fn dn_all(&self, t: N, n: uint, out: &mut [O]) {
        assert!(out.len() == n);

        for i in range(0, n) {
            out[i] = self.dn(t, i)
        }
    }
}

impl<'a, N: Copy, O, T: UnivariateFn<N, O>> UnivariateFn<N, O> for &'a T {
    #[inline]
    fn d0(&self, t: N) -> O {
        (*self).d0(t)
    }

    #[inline]
    fn ueval(&self, t: N) -> O {
        (*self).ueval(t)
    }

    #[inline]
    fn d1(&self, t: N) -> O {
        (*self).d1(t)
    }

    #[inline]
    fn d2(&self, t: N) -> O {
        (*self).d2(t)
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> O {
        (*self).dn(t, n)
    }

    #[inline]
    fn d0_1(&self, t: N) -> (O, O) {
        (*self).d0_1(t)
    }

    #[inline]
    fn d0_1_2(&self, t: N) -> (O, O, O) {
        (*self).d0_1_2(t)
    }

    #[inline]
    fn dn_all(&self, t: N, n: uint, out: &mut [O]) {
        (*self).dn_all(t, n, out)
    }
}

impl<'a, N: Copy, O> UnivariateFn<N, O> for &'a UnivariateFn<N, O> {
    #[inline]
    fn d0(&self, t: N) -> O {
        (*self).d0(t)
    }

    #[inline]
    fn ueval(&self, t: N) -> O {
        (*self).ueval(t)
    }

    #[inline]
    fn d1(&self, t: N) -> O {
        (*self).d1(t)
    }

    #[inline]
    fn d2(&self, t: N) -> O {
        (*self).d2(t)
    }

    #[inline]
    fn dn(&self, t: N, n: uint) -> O {
        (*self).dn(t, n)
    }

    #[inline]
    fn d0_1(&self, t: N) -> (O, O) {
        (*self).d0_1(t)
    }

    #[inline]
    fn d0_1_2(&self, t: N) -> (O, O, O) {
        (*self).d0_1_2(t)
    }

    #[inline]
    fn dn_all(&self, t: N, n: uint, out: &mut [O]) {
        (*self).dn_all(t, n, out)
    }
}
