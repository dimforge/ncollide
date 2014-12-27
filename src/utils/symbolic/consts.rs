use na::{Cast, Zero};
use na;
use utils::symbolic::{UnivariateFn, BivariateFn};

macro_rules! fn_impl(
    ($t: ident) => (
        impl<N: Copy, O: Zero + Cast<$t>> UnivariateFn<N, O> for $t {
            #[inline]
            fn d0(&self, _: N) -> O {
                na::cast(*self)
            }

            #[inline]
            fn d1(&self, _: N) -> O {
                na::zero()
            }

            #[inline]
            fn d2(&self, _: N) -> O {
                na::zero()
            }

            #[inline]
            fn dn(&self, t: N, n: uint) -> O {
                if n == 0 {
                    self.ueval(t)
                }
                else {
                    na::zero()
                }
            }
        }

        impl<N: Copy, O: Zero + Cast<$t> + Clone> BivariateFn<N, O> for $t {
            #[inline]
            fn d0(&self, _: N, _: N) -> O {
                na::cast(*self)
            }

            #[inline]
            fn du(&self, _: N, _: N) -> O {
                na::zero()
            }

            #[inline]
            fn dv(&self, _: N, _: N) -> O {
                na::zero()
            }

            #[inline]
            fn duu(&self, _: N, _: N) -> O {
                na::zero()
            }

            #[inline]
            fn dvv(&self, _: N, _: N) -> O {
                na::zero()
            }

            #[inline]
            fn duv(&self, _: N, _: N) -> O {
                na::zero()
            }

            #[inline]
            fn duv_nk(&self, u: N, v: N, n: uint, k: uint) -> O {
                if n == 0 && k == 0 {
                    self.beval(u, v)
                }
                else {
                    na::zero()
                }
            }
        }
    )
);

fn_impl!(f32);
fn_impl!(f64);
fn_impl!(u8);
fn_impl!(u16);
fn_impl!(u32);
fn_impl!(u64);
fn_impl!(i8);
fn_impl!(i16);
fn_impl!(i32);
fn_impl!(i64);
