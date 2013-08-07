//!
//! Support mapping based Minkowski Sum geometry.
//!

use std::num::{Zero, One};
use std::cmp::ApproxEq;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarDiv};
use nalgebra::traits::transformation::Transformable;
use geom::transformed::Transformed;
use geom::implicit::Implicit;
use geom::reflection::Reflection;

/// Type of an implicit representation of the Configuration Space Obstacle
/// formed by two geometric objects.
pub type CSO<'self, G1, G2> = MinkowskiSum<'self, G1, Reflection<'self, G2>>;
pub type AnnotatedCSO<'self, G1, G2> = AnnotatedMinkowskiSum<'self, G1, Reflection<'self, G2>>;

/**
 * Implicit representation of the minkowski sum of two geometries.
 * The only way to obtain the sum points is to use its support mapping
 * function.
 *
 *  - `G1`: type of the first object involved on the sum.
 *  - `G2`: type of the second object involved on the sum.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct MinkowskiSum<'self, G1, G2> {
    priv g1: &'self G1,
    priv g2: &'self G2
}

impl<'self, G1, G2> MinkowskiSum<'self, G1, G2> {
    /**
     * Builds the Minkowski sum of two geometries. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(g1: &'self G1, g2: &'self G2) -> MinkowskiSum<'self, G1, G2> {
        MinkowskiSum { g1: g1, g2: g2 }
    }
}

impl<'self, V: Add<V, V>, G1: Implicit<V>, G2: Implicit<V>>
Implicit<V> for MinkowskiSum<'self, G1, G2> {
    #[inline]
    fn support_point(&self, dir: &V) -> V {
        self.g1.support_point(dir) + self.g2.support_point(dir)
    }
}

/**
 * Same as the MinkowskiSum but with a support mapping which keeps track of the
 * original supports points from the two wrapped geometries.
 *  - `G1`: type of the first object involved on the sum.
 *  - `G2`: type of the second object involved on the sum.
 */
#[deriving(Eq, ToStr, Clone)]
pub struct AnnotatedMinkowskiSum<'self, G1, G2> {
    priv g1: &'self G1,
    priv g2: &'self G2
}

impl<'self, G1, G2> AnnotatedMinkowskiSum<'self, G1, G2> {
    /**
     * Builds the Minkowski sum of two geometries. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(g1: &'self G1, g2: &'self G2) -> AnnotatedMinkowskiSum<'self, G1, G2> {
        AnnotatedMinkowskiSum { g1: g1, g2: g2 }
    }
}

impl<'self, V: Add<V, V>, G1: Implicit<V>, G2: Implicit<V>>
Implicit<AnnotatedPoint<V>> for AnnotatedMinkowskiSum<'self, G1, G2> {
    #[inline]
    fn support_point(&self, dir: &AnnotatedPoint<V>) -> AnnotatedPoint<V> {
        let orig1 = self.g1.support_point(dir.point());
        let orig2 = self.g2.support_point(dir.point());
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

impl<V: Dot<N>, N> Dot<N> for AnnotatedPoint<V> {
    #[inline]
    fn dot(&self, other: &AnnotatedPoint<V>) -> N {
        self.point.dot(&other.point)
    }
}

impl<V: SubDot<N>, N> SubDot<N> for AnnotatedPoint<V> {
    #[inline]
    fn sub_dot(&self, sub: &AnnotatedPoint<V>, dot: &AnnotatedPoint<V>) -> N {
        self.point.sub_dot(&sub.point, &dot.point)
    }
}

impl<V: Norm<N> + Clone, N> Norm<N> for AnnotatedPoint<V> {
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

impl<V: ScalarDiv<N>, N> ScalarDiv<N> for AnnotatedPoint<V> {
    #[inline]
    fn scalar_div(&self, n: &N) -> AnnotatedPoint<V> {
        AnnotatedPoint::new(self.orig1.scalar_div(n),
        self.orig2.scalar_div(n),
        self.point.scalar_div(n))
    }

    #[inline]
    fn scalar_div_inplace(&mut self, n: &N) {
        self.orig1.scalar_div_inplace(n);
        self.orig2.scalar_div_inplace(n);
        self.point.scalar_div_inplace(n);
    }
}

impl<V: ScalarMul<N>, N> ScalarMul<N> for AnnotatedPoint<V> {
    #[inline]
    fn scalar_mul(&self, n: &N) -> AnnotatedPoint<V> {
        AnnotatedPoint::new(self.orig1.scalar_mul(n),
        self.orig2.scalar_mul(n),
        self.point.scalar_mul(n))
    }

    #[inline]
    fn scalar_mul_inplace(&mut self, n: &N) {
        self.orig1.scalar_mul_inplace(n);
        self.orig2.scalar_mul_inplace(n);
        self.point.scalar_mul_inplace(n);
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
pub fn cso_support_point<G1: Implicit<V>,
                         G2: Implicit<V>,
                         V:  Zero + Neg<V> + Add<V, V>>(
                         g1:  &G1,
                         g2:  &G2,
                         dir: V)
                         -> AnnotatedPoint<V> {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(g1, &rg2);

    cso.support_point(&AnnotatedPoint::new_invalid(dir))
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

impl<'self, G1: Clone, G2: Clone, M: Clone + Mul<M, M> + Inv, N>
Transformable<M, Transformed<MinkowskiSum<'self, G1, G2>, M, N>> for MinkowskiSum<'self, G1, G2> {
    #[inline]
    fn transformed(&self, transform: &M) -> Transformed<MinkowskiSum<'self, G1, G2>, M, N> {
        Transformed::new(transform.clone(), self.clone())
    }
}
