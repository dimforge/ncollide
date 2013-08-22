//!
//! Support mapping based Minkowski Sum geometry. Note thas the support mapping function will
//! ignore the transformation matrix (the first argument of the `support_point` method).
//!

use std::num::{Zero, One};
use std::cmp::ApproxEq;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::vector::{Vec, AlgebraicVec};
use geom::implicit::Implicit;
use geom::reflection::Reflection;

/// Type of an implicit representation of the Configuration Space Obstacle
/// formed by two geometric objects.
pub type CSO<'self, M, G1, G2> = NonTransformableMinkowskiSum<'self, M, G1, Reflection<'self, G2>>;
pub type AnnotatedCSO<'self, M, G1, G2> = AnnotatedNonTransformableMinkowskiSum<'self, M, G1, Reflection<'self, G2>>;

/**
 * Implicit representation of the minkowski sum of two geometries.
 * The only way to obtain the sum points is to use its support mapping
 * function.
 *
 *  - `G1`: type of the first object involved on the sum.
 *  - `G2`: type of the second object involved on the sum.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct NonTransformableMinkowskiSum<'self, M, G1, G2> {
    priv m1: &'self M,
    priv g1: &'self G1,
    priv m2: &'self M,
    priv g2: &'self G2
}

impl<'self, M, G1, G2> NonTransformableMinkowskiSum<'self, M, G1, G2> {
    /**
     * Builds the Minkowski sum of two geometries. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(m1: &'self M,
               g1: &'self G1,
               m2: &'self M,
               g2: &'self G2)
               -> NonTransformableMinkowskiSum<'self, M, G1, G2> {
        NonTransformableMinkowskiSum { m1: m1, g1: g1, m2: m2, g2: g2 }
    }
}

impl<'self, N: Num + Algebraic, V: AlgebraicVec<N>, M, G1: Implicit<N, V, M>, G2: Implicit<N, V, M>>
Implicit<N, V, M> for NonTransformableMinkowskiSum<'self, M, G1, G2> {
    #[inline]
    fn margin(&self) -> N {
        self.g1.margin() + self.g2.margin()
    }

    #[inline]
    fn support_point(&self, _: &M, dir: &V) -> V {
        self.g1.support_point(self.m1, dir) + self.g2.support_point(self.m2, dir)
    }

    #[inline]
    fn support_point_without_margin(&self, _: &M, dir: &V) -> V {
        self.g1.support_point_without_margin(self.m1, dir) +
        self.g2.support_point_without_margin(self.m2, dir)
    }
}

/**
 * Same as the NonTransformableMinkowskiSum but with a support mapping which keeps track of the
 * original supports points from the two wrapped geometries.
 *  - `G1`: type of the first object involved on the sum.
 *  - `G2`: type of the second object involved on the sum.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct AnnotatedNonTransformableMinkowskiSum<'self, M, G1, G2> {
    priv m1: &'self M,
    priv g1: &'self G1,
    priv m2: &'self M,
    priv g2: &'self G2
}

impl<'self, M, G1, G2> AnnotatedNonTransformableMinkowskiSum<'self, M, G1, G2> {
    /**
     * Builds the Minkowski sum of two geometries. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(m1: &'self M,
               g1: &'self G1,
               m2: &'self M,
               g2: &'self G2) -> AnnotatedNonTransformableMinkowskiSum<'self, M, G1, G2> {
        AnnotatedNonTransformableMinkowskiSum { m1: m1, g1: g1, m2: m2, g2: g2 }
    }
}

impl<'self, N: Algebraic + Num, V: AlgebraicVec<N> + Clone, M, G1: Implicit<N, V, M>, G2: Implicit<N, V, M>>
Implicit<N, AnnotatedPoint<V>, M> for AnnotatedNonTransformableMinkowskiSum<'self, M, G1, G2> {
    #[inline]
    fn margin(&self) -> N {
        self.g1.margin() + self.g2.margin()
    }

    #[inline]
    fn support_point(&self, _: &M, dir: &AnnotatedPoint<V>) -> AnnotatedPoint<V> {
        let orig1 = self.g1.support_point(self.m1, dir.point());
        let orig2 = self.g2.support_point(self.m2, dir.point());
        let point = orig1 + orig2;

        AnnotatedPoint::new(orig1, orig2, point)
    }

    #[inline]
    fn support_point_without_margin(&self, _: &M, dir: &AnnotatedPoint<V>) -> AnnotatedPoint<V> {
        let orig1 = self.g1.support_point_without_margin(self.m1, dir.point());
        let orig2 = self.g2.support_point_without_margin(self.m2, dir.point());
        let point = orig1 + orig2;

        AnnotatedPoint::new(orig1, orig2, point)
    }
}

// FIXME: AnnotatedPoint is not a good name.
#[doc(hidden)]
#[deriving(Clone, ToStr)]
pub struct AnnotatedPoint<V> {
    priv orig1: V,
    priv orig2: V,
    priv point: V
}

impl<V> AnnotatedPoint<V> {
    #[doc(hidden)]
    #[inline]
    pub fn new(orig1: V, orig2: V, point: V) -> AnnotatedPoint<V> {
        AnnotatedPoint {
            orig1: orig1,
            orig2: orig2,
            point: point
        }
    }

    #[doc(hidden)]
    #[inline]
    pub fn point<'r>(&'r self) -> &'r V {
        &'r self.point
    }

    #[doc(hidden)]
    #[inline]
    pub fn orig1<'r>(&'r self) -> &'r V {
        &'r self.orig1
    }

    #[doc(hidden)]
    #[inline]
    pub fn orig2<'r>(&'r self) -> &'r V {
        &'r self.orig2
    }
}

impl<V: Zero> AnnotatedPoint<V> {
    #[doc(hidden)]
    #[inline]
    pub fn new_invalid(point: V) -> AnnotatedPoint<V> {
        AnnotatedPoint {
            orig1: Zero::zero::(),
            orig2: Zero::zero::(),
            point: point
        }
    }
}


impl<V: Zero> Zero for AnnotatedPoint<V> {
    #[inline]
    fn zero() -> AnnotatedPoint<V> {
        AnnotatedPoint::new(Zero::zero(), Zero::zero(), Zero::zero())
    }

    #[inline]
    fn is_zero(&self) -> bool {
        self.point.is_zero()
    }
}

impl<V: One> One for AnnotatedPoint<V> {
    // FIXME: this definition works but is flawed (orig1 + orig2 != point)
    #[inline]
    fn one() -> AnnotatedPoint<V> {
        AnnotatedPoint::new(One::one(), One::one(), One::one())
    }
}

impl<V: Sub<V, V>> Sub<AnnotatedPoint<V>, AnnotatedPoint<V>> for
AnnotatedPoint<V> {
    #[inline]
    fn sub(&self, other: &AnnotatedPoint<V>) -> AnnotatedPoint<V> {
        AnnotatedPoint::new(self.orig1 - other.orig1,
        self.orig2 - other.orig2,
        self.point - other.point)
    }
}

impl<V: Add<V, V>> Add<AnnotatedPoint<V>, AnnotatedPoint<V>> for
AnnotatedPoint<V> {
    #[inline]
    fn add(&self, other: &AnnotatedPoint<V>) -> AnnotatedPoint<V> {
        AnnotatedPoint::new(self.orig1 + other.orig1,
        self.orig2 + other.orig2,
        self.point + other.point)
    }
}

impl<V: Neg<V>> Neg<AnnotatedPoint<V>> for AnnotatedPoint<V> {
    #[inline]
    fn neg(&self) -> AnnotatedPoint<V> {
        AnnotatedPoint::new(-self.orig1, -self.orig2, -self.point)
    }
}

impl<V: Dim> Dim for AnnotatedPoint<V> {
    #[inline]
    fn dim() -> uint {
        Dim::dim::<V>()
    }
}

impl<V: Vec<N>, N> Vec<N> for AnnotatedPoint<V> {
    #[inline]
    fn dot(&self, other: &AnnotatedPoint<V>) -> N {
        self.point.dot(&other.point)
    }

    #[inline]
    fn sub_dot(&self, sub: &AnnotatedPoint<V>, dot: &AnnotatedPoint<V>) -> N {
        self.point.sub_dot(&sub.point, &dot.point)
    }
}

impl<N: Algebraic, V: AlgebraicVec<N> + Clone> AlgebraicVec<N> for AnnotatedPoint<V> {
    #[inline]
    fn norm(&self) -> N {
        self.point.norm()
    }

    #[inline]
    fn sqnorm(&self) -> N {
        self.point.sqnorm()
    }

    /// Be careful: only the `point` is normalized, not `orig1` nor `orig2`.
    #[inline]
    fn normalized(&self) -> AnnotatedPoint<V> {
        AnnotatedPoint::new(self.orig1.clone(), self.orig2.clone(), self.point.normalized())
    }

    /// Be careful: only the `point` is normalized, not `orig1` nor `orig2`.
    #[inline]
    fn normalize(&mut self) -> N {
        self.point.normalize()
    }
}

impl<V: Div<N, V>, N> Div<N, AnnotatedPoint<V>> for AnnotatedPoint<V> {
    #[inline]
    fn div(&self, n: &N) -> AnnotatedPoint<V> {
        AnnotatedPoint::new(self.orig1 / *n, self.orig2 / *n, self.point / *n)
    }
}

impl<V: Mul<N, V>, N> Mul<N, AnnotatedPoint<V>> for AnnotatedPoint<V> {
    #[inline]
    fn mul(&self, n: &N) -> AnnotatedPoint<V> {
        AnnotatedPoint::new(self.orig1 * *n, self.orig2 * *n, self.point * *n)
    }
}

impl<V: Eq> Eq for AnnotatedPoint<V> {
    #[inline]
    fn eq(&self, other: &AnnotatedPoint<V>) -> bool {
        self.point == other.point
    }

    #[inline]
    fn ne(&self, other: &AnnotatedPoint<V>) -> bool {
        self.point != other.point
    }
}

/// Computes the support point of a CSO on a given direction.
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point<G1: Implicit<N, V, M>,
                         G2: Implicit<N, V, M>,
                         N:  Algebraic + Num,
                         V:  AlgebraicVec<N> + Clone,
                         M>(
                         m1:  &M,
                         g1:  &G1,
                         m2:  &M,
                         g2:  &G2,
                         dir: V)
                         -> AnnotatedPoint<V> {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedNonTransformableMinkowskiSum::new(m1, g1, m2, &rg2);

    // m1 or whatever: it will be ignored
    cso.support_point(m1, &AnnotatedPoint::new_invalid(dir))
}

/// Computes the support point of a CSO on a given direction.
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point_without_margin<G1: Implicit<N, V, M>,
                                        G2: Implicit<N, V, M>,
                                        N:  Algebraic + Num,
                                        V:  AlgebraicVec<N> + Clone,
                                        M>(
                                        m1:  &M,
                                        g1:  &G1,
                                        m2:  &M,
                                        g2:  &G2,
                                        dir: V)
                                        -> AnnotatedPoint<V> {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedNonTransformableMinkowskiSum::new(m1, g1, m2, &rg2);

    // m1 or whatever: it will be ignored
    cso.support_point_without_margin(m1, &AnnotatedPoint::new_invalid(dir))
}

impl<V: ApproxEq<N>, N: ApproxEq<N>> ApproxEq<N> for AnnotatedPoint<V> {
    #[inline]
    fn approx_epsilon() -> N {
        ApproxEq::approx_epsilon::<N, N>()
    }

    #[inline]
    fn approx_eq(&self, other: &AnnotatedPoint<V>) -> bool {
        self.point.approx_eq(&other.point)
    }

    #[inline]
    fn approx_eq_eps(&self, other: &AnnotatedPoint<V>, epsilon: &N) -> bool {
        self.point.approx_eq_eps(&other.point, epsilon)
    }
}
