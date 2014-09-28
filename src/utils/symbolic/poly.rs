use std::num::{Zero, One};
use na::Cast;
use na;
use utils::symbolic::{UnivariateFn, SymAdd, SymMult, SymSub, SymNeg, SymComp};
use utils::symbolic;

macro_rules! decl_poly(
    ($t: ident, $dim: expr, $t1: ident, $comp0: ident $(,$compN: ident)*) => (
        /// A polynomial function.
        pub struct $t<N> {
            coeffs: [N, ..$dim]
        }

        impl<N: Copy> Clone for $t<N> {
            fn clone(&self) -> $t<N> {
                let mut new_coeffs: [N, ..$dim] = [self.coeffs[0], ..$dim];

                for i in range(1, $dim) {
                    new_coeffs[i] = self.coeffs[i];
                }

                $t { coeffs: new_coeffs }
            }
        }

        /// A polynomial function.
        #[inline]
        pub fn $t1<N>($comp0: N $( , $compN: N)*) -> $t<N> {
            $t { coeffs: [
                $comp0 $(, $compN )*
                ]}
        }

        impl<N: Copy + One + Zero + Add<N, N> + Mul<N, N> + Cast<f64>> UnivariateFn<N, N> for $t<N> {
            #[inline]
            fn d0(&self, t: N) -> N {
                self.dn(t, 0)
            }

            #[inline]
            fn d1(&self, t: N) -> N {
                self.dn(t, 1)
            }

            #[inline]
            fn d2(&self, t: N) -> N {
                self.dn(t, 2)
            }

            #[inline]
            fn dn(&self, t: N, n: uint) -> N {
                let mut res = na::zero::<N>();
                let mut pow = na::one();

                // FIXME: this is not a very stable exponentiation method.
                for (power, coeff) in self.coeffs.slice_from(n).iter().enumerate() {
                    let mut dpow: uint = na::one();

                    for i in range(0, n - power) {
                        dpow = dpow * (n - i)
                    }

                    res = res + *coeff * na::cast(dpow as f64) * pow;
                    pow = pow * t;
                }

                res
            }
        }

        impl<A: Copy, B: Clone> Add<B, SymAdd<$t<A>, B>> for $t<A> {
            fn add(&self, other: &B) -> SymAdd<$t<A>, B> {
                symbolic::add(self.clone(), other.clone())
            }
        }
        
        impl<A: Copy, B: Clone> Sub<B, SymSub<$t<A>, B>> for $t<A> {
            fn sub(&self, other: &B) -> SymSub<$t<A>, B> {
                symbolic::sub(self.clone(), other.clone())
            }
        }
        
        impl<A: Copy, B: Clone> Mul<B, SymMult<$t<A>, B>> for $t<A> {
            fn mul(&self, other: &B) -> SymMult<$t<A>, B> {
                symbolic::mult(self.clone(), other.clone())
            }
        }
        
        impl<A: Copy> Neg<SymNeg<$t<A>>> for $t<A> {
            fn neg(&self) -> SymNeg<$t<A>> {
                symbolic::neg(self.clone())
            }
        }
        
        // XXX: replace by `Fn` when method call overloading will be supported by Rust.
        impl<A: Copy, B: Clone> BitAnd<B, SymComp<$t<A>, B>> for $t<A> {
            fn bitand(&self, other: &B) -> SymComp<$t<A>, B> {
                symbolic::comp(self.clone(), other.clone())
            }
        }
    )
)

decl_poly!(Poly1, 1, t1, c1)
decl_poly!(Poly2, 2, t2, c1, c2)
decl_poly!(Poly3, 3, t3, c1, c2, c3)
decl_poly!(Poly4, 4, t4, c1, c2, c3, c4)
decl_poly!(Poly5, 5, t5, c1, c2, c3, c4, c5)
decl_poly!(Poly6, 6, t6, c1, c2, c3, c4, c5, c6)
