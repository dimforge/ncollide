#![macro_escape]

macro_rules! impl_ops(
    ($t: ident) => (
        impl<A: Clone, B: Clone> Add<B, SymAdd<$t<A>, B>> for $t<A> {
            #[allow(unnecessary_qualification)]
            fn add(&self, other: &B) -> SymAdd<$t<A>, B> {
                symbolic::add(self.clone(), other.clone())
            }
        }

        impl<A: Clone, B: Clone> Sub<B, SymSub<$t<A>, B>> for $t<A> {
            #[allow(unnecessary_qualification)]
            fn sub(&self, other: &B) -> SymSub<$t<A>, B> {
                symbolic::sub(self.clone(), other.clone())
            }
        }

        impl<A: Clone, B: Clone> Mul<B, SymMult<$t<A>, B>> for $t<A> {
            #[allow(unnecessary_qualification)]
            fn mul(&self, other: &B) -> SymMult<$t<A>, B> {
                symbolic::mult(self.clone(), other.clone())
            }
        }

        impl<A: Clone> Neg<SymNeg<$t<A>>> for $t<A> {
            #[allow(unnecessary_qualification)]
            fn neg(&self) -> SymNeg<$t<A>> {
                symbolic::neg(self.clone())
            }
        }
    )
)

macro_rules! impl_ops_bin(
    ($t: ident) => (
        impl<A: Clone, B: Clone, C: Clone> Add<C, SymAdd<$t<A, B>, C>> for $t<A, B> {
            #[allow(unnecessary_qualification)]
            fn add(&self, other: &C) -> SymAdd<$t<A, B>, C> {
                symbolic::add(self.clone(), other.clone())
            }
        }

        impl<A: Clone, B: Clone, C: Clone> Sub<C, SymSub<$t<A, B>, C>> for $t<A, B> {
            #[allow(unnecessary_qualification)]
            fn sub(&self, other: &C) -> SymSub<$t<A, B>, C> {
                symbolic::sub(self.clone(), other.clone())
            }
        }

        impl<A: Clone, B: Clone, C: Clone> Mul<C, SymMult<$t<A, B>, C>> for $t<A, B> {
            #[allow(unnecessary_qualification)]
            fn mul(&self, other: &C) -> SymMult<$t<A, B>, C> {
                symbolic::mult(self.clone(), other.clone())
            }
        }

        impl<A: Clone, B: Clone> Neg<SymNeg<$t<A, B>>> for $t<A, B> {
            #[allow(unnecessary_qualification)]
            fn neg(&self) -> SymNeg<$t<A, B>> {
                symbolic::neg(self.clone())
            }
        }
    )
)

macro_rules! impl_ops_noparam(
    ($t: ident) => (
        impl<C: Clone> Add<C, SymAdd<$t, C>> for $t {
            #[allow(unnecessary_qualification)]
            fn add(&self, other: &C) -> SymAdd<$t, C> {
                symbolic::add(self.clone(), other.clone())
            }
        }

        impl<C: Clone> Sub<C, SymSub<$t, C>> for $t {
            #[allow(unnecessary_qualification)]
            fn sub(&self, other: &C) -> SymSub<$t, C> {
                symbolic::sub(self.clone(), other.clone())
            }
        }

        impl<C: Clone> Mul<C, SymMult<$t, C>> for $t {
            #[allow(unnecessary_qualification)]
            fn mul(&self, other: &C) -> SymMult<$t, C> {
                symbolic::mult(self.clone(), other.clone())
            }
        }

        impl Neg<SymNeg<$t>> for $t {
            #[allow(unnecessary_qualification)]
            fn neg(&self) -> SymNeg<$t> {
                symbolic::neg(self.clone())
            }
        }
    )
)
