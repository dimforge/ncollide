#![macro_escape]

macro_rules! impl_ops(
    ($t: ident) => (
        impl<A, B> Add<B, SymAdd<$t<A>, B>> for $t<A> {
            #[allow(unused_qualifications)]
            fn add(self, other: B) -> SymAdd<$t<A>, B> {
                symbolic::add(self, other)
            }
        }

        impl<A, B> Sub<B, SymSub<$t<A>, B>> for $t<A> {
            #[allow(unused_qualifications)]
            fn sub(self, other: B) -> SymSub<$t<A>, B> {
                symbolic::sub(self, other)
            }
        }

        impl<A, B> Mul<B, SymMult<$t<A>, B>> for $t<A> {
            #[allow(unused_qualifications)]
            fn mul(self, other: B) -> SymMult<$t<A>, B> {
                symbolic::mult(self, other)
            }
        }

        impl<A: Clone> Neg<SymNeg<$t<A>>> for $t<A> {
            #[allow(unused_qualifications)]
            fn neg(&self) -> SymNeg<$t<A>> {
                symbolic::neg(self.clone())
            }
        }
    )
);

macro_rules! impl_ops_bin(
    ($t: ident) => (
        impl<A, B, C> Add<C, SymAdd<$t<A, B>, C>> for $t<A, B> {
            #[allow(unused_qualifications)]
            fn add(self, other: C) -> SymAdd<$t<A, B>, C> {
                symbolic::add(self, other)
            }
        }

        impl<A, B, C> Sub<C, SymSub<$t<A, B>, C>> for $t<A, B> {
            #[allow(unused_qualifications)]
            fn sub(self, other: C) -> SymSub<$t<A, B>, C> {
                symbolic::sub(self, other)
            }
        }

        impl<A, B, C> Mul<C, SymMult<$t<A, B>, C>> for $t<A, B> {
            #[allow(unused_qualifications)]
            fn mul(self, other: C) -> SymMult<$t<A, B>, C> {
                symbolic::mult(self, other)
            }
        }

        impl<A: Clone, B: Clone> Neg<SymNeg<$t<A, B>>> for $t<A, B> {
            #[allow(unused_qualifications)]
            fn neg(&self) -> SymNeg<$t<A, B>> {
                symbolic::neg(self.clone())
            }
        }
    )
);

macro_rules! impl_ops_noparam(
    ($t: ident) => (
        impl<C> Add<C, SymAdd<$t, C>> for $t {
            #[allow(unused_qualifications)]
            fn add(self, other: C) -> SymAdd<$t, C> {
                symbolic::add(self, other)
            }
        }

        impl<C> Sub<C, SymSub<$t, C>> for $t {
            #[allow(unused_qualifications)]
            fn sub(self, other: C) -> SymSub<$t, C> {
                symbolic::sub(self, other)
            }
        }

        impl<C> Mul<C, SymMult<$t, C>> for $t {
            #[allow(unused_qualifications)]
            fn mul(self, other: C) -> SymMult<$t, C> {
                symbolic::mult(self, other)
            }
        }

        impl Neg<SymNeg<$t>> for $t {
            #[allow(unused_qualifications)]
            fn neg(&self) -> SymNeg<$t> {
                symbolic::neg(self.clone())
            }
        }
    )
);
