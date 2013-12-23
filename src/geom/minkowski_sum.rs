//!
//! Support mapping based Minkowski Sum geometry. Note thas the support mapping function will
//! ignore the transformation matrix (the first argument of the `support_point` method).
//!

use std::num::{Zero, One};
use std::cmp::ApproxEq;
use nalgebra::na::{Dot, Norm, Vec, Dim};
use nalgebra::na;
use geom::Reflection;

/// Type of an implicit representation of the Configuration Space Obstacle
/// formed by two geometric objects.
pub type CSO<'a, M, G1, G2> = MinkowskiSum<'a, M, G1, Reflection<'a, G2>>;
pub type AnnotatedCSO<'a, M, G1, G2> = AnnotatedMinkowskiSum<'a, M, G1, Reflection<'a, G2>>;

/**
 * Implicit representation of the minkowski sum of two geometries.
 * The only way to obtain the sum points is to use its support mapping
 * function.
 *
 *  - `G1`: type of the first object involved on the sum.
 *  - `G2`: type of the second object involved on the sum.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct MinkowskiSum<'a, M, G1, G2> {
    priv m1: &'a M,
    priv g1: &'a G1,
    priv m2: &'a M,
    priv g2: &'a G2
}

impl<'a, M, G1, G2> MinkowskiSum<'a, M, G1, G2> {
    /**
     * Builds the Minkowski sum of two geometries. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(m1: &'a M,
               g1: &'a G1,
               m2: &'a M,
               g2: &'a G2)
               -> MinkowskiSum<'a, M, G1, G2> {
        MinkowskiSum { m1: m1, g1: g1, m2: m2, g2: g2 }
    }

    #[inline]
    pub fn m1(&self) -> &'a M {
        self.m1
    }

    #[inline]
    pub fn m2(&self) -> &'a M {
        self.m2
    }

    #[inline]
    pub fn g1(&self) -> &'a G1 {
        self.g1
    }

    #[inline]
    pub fn g2(&self) -> &'a G2 {
        self.g2
    }
}

/**
 * Same as the MinkowskiSum but with a support mapping which keeps track of the
 * original supports points from the two wrapped geometries.
 *  - `G1`: type of the first object involved on the sum.
 *  - `G2`: type of the second object involved on the sum.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct AnnotatedMinkowskiSum<'a, M, G1, G2> {
    priv m1: &'a M,
    priv g1: &'a G1,
    priv m2: &'a M,
    priv g2: &'a G2
}

impl<'a, M, G1, G2> AnnotatedMinkowskiSum<'a, M, G1, G2> {
    /**
     * Builds the Minkowski sum of two geometries. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(m1: &'a M,
               g1: &'a G1,
               m2: &'a M,
               g2: &'a G2) -> AnnotatedMinkowskiSum<'a, M, G1, G2> {
        AnnotatedMinkowskiSum { m1: m1, g1: g1, m2: m2, g2: g2 }
    }

    #[inline]
    pub fn m1(&self) -> &'a M {
        self.m1
    }

    #[inline]
    pub fn m2(&self) -> &'a M {
        self.m2
    }

    #[inline]
    pub fn g1(&self) -> &'a G1 {
        self.g1
    }

    #[inline]
    pub fn g2(&self) -> &'a G2 {
        self.g2
    }
}

// FIXME: AnnotatedPoint is not a good name.
#[doc(hidden)]
#[deriving(Clone, ToStr, Encodable, Decodable)]
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
            orig1: na::zero(),
            orig2: na::zero(),
            point: point
        }
    }
}


impl<V: Zero> Zero for AnnotatedPoint<V> {
    #[inline]
    fn zero() -> AnnotatedPoint<V> {
        AnnotatedPoint::new(na::zero(), na::zero(), na::zero())
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
    fn dim(_: Option<AnnotatedPoint<V>>) -> uint {
        na::dim::<V>()
    }
}

impl<V: Vec<N>, N> Dot<N> for AnnotatedPoint<V> {
    #[inline]
    fn dot(a: &AnnotatedPoint<V>, b: &AnnotatedPoint<V>) -> N {
        na::dot(&a.point, &b.point)
    }

    #[inline]
    fn sub_dot(a: &AnnotatedPoint<V>, b: &AnnotatedPoint<V>, c: &AnnotatedPoint<V>) -> N {
        na::sub_dot(&a.point, &b.point, &c.point)
    }
}

impl<N: Algebraic, V: Norm<N> + Clone> Norm<N> for AnnotatedPoint<V> {
    #[inline]
    fn norm(v: &AnnotatedPoint<V>) -> N {
        na::norm(&v.point)
    }

    #[inline]
    fn sqnorm(v: &AnnotatedPoint<V>) -> N {
        na::sqnorm(&v.point)
    }

    /// Be careful: only the `point` is normalized, not `orig1` nor `orig2`.
    #[inline]
    fn normalize_cpy(v: &AnnotatedPoint<V>) -> AnnotatedPoint<V> {
        AnnotatedPoint::new(v.orig1.clone(), v.orig2.clone(), na::normalize(&v.point))
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

impl<V: ApproxEq<N>, N: ApproxEq<N>> ApproxEq<N> for AnnotatedPoint<V> {
    #[inline]
    fn approx_epsilon() -> N {
        fail!("approx_epsilon is broken since rust revision 8693943676487c01fa09f5f3daf0df6a1f71e24d.")
        // ApproxEq::<N>::approx_epsilon()
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
