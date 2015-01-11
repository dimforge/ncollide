//! Minkowski sum.

use std::ops::{Index, IndexMut, Add, Sub, Mul, Div, Neg};
use na::{Dim, ApproxEq, Orig, PntAsVec, Axpy, Translate, NumPnt, NumVec, POrd,
         POrdering, ScalarSub, ScalarAdd, ScalarMul, ScalarDiv, FloatPnt, Bounded};
use na;
use shape::Reflection;
use math::{Scalar, Point, Vect};


/// Type of an implicit representation of the Configuration Space Obstacle
/// formed by two geometric objects.
pub type CSO<'a, M, G1, G2> = MinkowskiSum<'a, M, G1, Reflection<'a, G2>>;
pub type AnnotatedCSO<'a, M, G1, G2> = AnnotatedMinkowskiSum<'a, M, G1, Reflection<'a, G2>>;

/**
 * SupportMap representation of the Minkowski sum of two shapes.
 *
 * The only way to obtain the sum points is to use its support mapping
 * function.
 *
 *  - `G1`: type of the first object involved on the sum.
 *  - `G2`: type of the second object involved on the sum.
 */
#[derive(Show)]
pub struct MinkowskiSum<'a, M: 'a, G1: ?Sized + 'a, G2: ?Sized + 'a> {
    m1: &'a M,
    g1: &'a G1,
    m2: &'a M,
    g2: &'a G2
}

impl<'a, M, G1: ?Sized, G2: ?Sized> MinkowskiSum<'a, M, G1, G2> {
    /**
     * Builds the Minkowski sum of two shapes. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(m1: &'a M, g1: &'a G1, m2: &'a M, g2: &'a G2) -> MinkowskiSum<'a, M, G1, G2> {
        MinkowskiSum { m1: m1, g1: g1, m2: m2, g2: g2 }
    }

    /// The transformation matrix of the first shape of this Minkowski Sum.
    #[inline]
    pub fn m1(&self) -> &'a M {
        self.m1
    }

    /// The transformation matrix of the second shape of this Minkowski Sum.
    #[inline]
    pub fn m2(&self) -> &'a M {
        self.m2
    }

    /// The first shape of this Minkowski Sum.
    #[inline]
    pub fn g1(&self) -> &'a G1 {
        self.g1
    }

    /// The second shape of this Minkowski Sum.
    #[inline]
    pub fn g2(&self) -> &'a G2 {
        self.g2
    }
}

/**
 * Same as the MinkowskiSum but with a support mapping which keeps track of the
 * original supports points from the two wrapped shapes.
 *
 * * `G1`: type of the first object involved on the sum.
 * * `G2`: type of the second object involved on the sum.
 */
#[derive(Show)]
pub struct AnnotatedMinkowskiSum<'a, M: 'a, G1: ?Sized + 'a, G2: ?Sized + 'a> {
    m1: &'a M,
    g1: &'a G1,
    m2: &'a M,
    g2: &'a G2
}

impl<'a, M, G1: ?Sized, G2: ?Sized> AnnotatedMinkowskiSum<'a, M, G1, G2> {
    /**
     * Builds the Minkowski sum of two shapes. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(m1: &'a M, g1: &'a G1, m2: &'a M, g2: &'a G2) -> AnnotatedMinkowskiSum<'a, M, G1, G2> {
        AnnotatedMinkowskiSum { m1: m1, g1: g1, m2: m2, g2: g2 }
    }

    /// The transformation matrix of the first shape of this Minkowski Sum.
    #[inline]
    pub fn m1(&self) -> &'a M {
        self.m1
    }

    /// The transformation matrix of the second shape of this Minkowski Sum.
    #[inline]
    pub fn m2(&self) -> &'a M {
        self.m2
    }

    /// The first shape of this Minkowski Sum.
    #[inline]
    pub fn g1(&self) -> &'a G1 {
        self.g1
    }

    /// The second shape of this Minkowski Sum.
    #[inline]
    pub fn g2(&self) -> &'a G2 {
        self.g2
    }
}

// FIXME: AnnotatedPoint is not a good name.
// XXX: do not hide the documentation!
#[doc(hidden)]
#[derive(Clone, Copy, Show, RustcEncodable, RustcDecodable)]
pub struct AnnotatedPoint<P> {
    orig1: P,
    orig2: P,
    point: P
}

impl<P> AnnotatedPoint<P> {
    #[doc(hidden)]
    #[inline]
    pub fn new(orig1: P, orig2: P, point: P) -> AnnotatedPoint<P> {
        AnnotatedPoint {
            orig1: orig1,
            orig2: orig2,
            point: point
        }
    }

    #[doc(hidden)]
    #[inline]
    pub fn point<'r>(&'r self) -> &'r P {
        &self.point
    }

    #[doc(hidden)]
    #[inline]
    pub fn orig1(&self) -> &P {
        &self.orig1
    }

    #[doc(hidden)]
    #[inline]
    pub fn orig2(&self) -> &P {
        &self.orig2
    }

    #[doc(hidden)]
    #[inline]
    pub fn translate_1<V: Translate<P>>(&mut self, t: &V) {
        self.orig1 = na::translate(t, &self.orig1);
        self.point = na::translate(t, &self.point);
    }

    #[doc(hidden)]
    #[inline]
    pub fn translate_2<V: Translate<P>>(&mut self, t: &V) {
        self.orig2 = na::translate(t, &self.orig2);
        self.point = na::translate(t, &self.point);
    }
}

impl<P: PntAsVec<V>, V> PntAsVec<V> for AnnotatedPoint<P> {
    #[inline]
    fn to_vec(self) -> V {
        self.point.to_vec()
    }

    #[inline]
    fn as_vec<'a>(&'a self) -> &'a V {
        self.point.as_vec()
    }

    #[inline]
    fn set_coords(&mut self, _: V) {
        panic!(".set_coords is not implemented for annotated points.")
    }
}

impl<P: Index<usize>> Index<usize> for AnnotatedPoint<P> {
    type Output = P::Output;

    #[inline]
    fn index(&self, i: &usize) -> &P::Output {
        &self.point[*i]
    }
}

impl<P: IndexMut<usize>> IndexMut<usize> for AnnotatedPoint<P> {
    type Output = P::Output;

    #[inline]
    fn index_mut(&mut self, _: &usize) -> &mut P::Output {
        unimplemented!()
    }
}

impl<P> POrd for AnnotatedPoint<P> {
    fn inf(&self, _: &AnnotatedPoint<P>) -> AnnotatedPoint<P> {
        unimplemented!()
    }

    fn sup(&self, _: &AnnotatedPoint<P>) -> AnnotatedPoint<P> {
        unimplemented!()
    }

    fn partial_cmp(&self, _: &AnnotatedPoint<P>) -> POrdering {
        unimplemented!()
    }
}

impl<P: Orig> Orig for AnnotatedPoint<P> {
    #[inline]
    fn orig() -> AnnotatedPoint<P> {
        AnnotatedPoint::new(na::orig(), na::orig(), na::orig())
    }

    #[inline]
    fn is_orig(&self) -> bool {
        self.point.is_orig()
    }
}

#[old_impl_check]
impl<N, P, V> Add<V> for AnnotatedPoint<P>
    where N: Scalar,
          P: Add<V, Output = P>,
          V: Copy + Mul<N, Output = V> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn add(self, other: V) -> AnnotatedPoint<P> {
        let _0_5: N = na::cast(0.5f64);

        AnnotatedPoint::new(
            self.orig1 + other * _0_5,
            self.orig2 + other * _0_5,
            self.point + other
        )
    }
}

impl<N, P: Axpy<N>> Axpy<N> for AnnotatedPoint<P> {
    #[inline]
    fn axpy(&mut self, a: &N, x: &AnnotatedPoint<P>) {
        self.orig1.axpy(a, &x.orig1);
        self.orig2.axpy(a, &x.orig2);
        self.point.axpy(a, &x.point);
    }
}

impl<P: Sub<P>> Sub<AnnotatedPoint<P>> for AnnotatedPoint<P> {
    type Output = P::Output;
    #[inline]
    fn sub(self, other: AnnotatedPoint<P>) -> P::Output {
        self.point - other.point
    }
}

#[old_impl_check]
impl<N, P, V> ScalarSub<N> for AnnotatedPoint<P>
    where P: Point<N, V> {
    fn sub_s(&self, _: &N) -> AnnotatedPoint<P> {
        unimplemented!()
    }
}

#[old_impl_check]
impl<N, P, V> ScalarAdd<N> for AnnotatedPoint<P>
    where P: Point<N, V> {
    fn add_s(&self, _: &N) -> AnnotatedPoint<P> {
        unimplemented!()
    }
}

#[old_impl_check]
impl<N, P, V> ScalarMul<N> for AnnotatedPoint<P>
    where P: Point<N, V> {
    fn mul_s(&self, _: &N) -> AnnotatedPoint<P> {
        unimplemented!()
    }
}

#[old_impl_check]
impl<N, P, V> ScalarDiv<N> for AnnotatedPoint<P>
    where P: Point<N, V> {
    fn div_s(&self, _: &N) -> AnnotatedPoint<P> {
        unimplemented!()
    }
}

impl<P: Neg<Output = P>> Neg for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn neg(self) -> AnnotatedPoint<P> {
        AnnotatedPoint::new(-self.orig1, -self.orig2, -self.point)
    }
}

impl<P: Dim> Dim for AnnotatedPoint<P> {
    #[inline]
    fn dim(_: Option<AnnotatedPoint<P>>) -> usize {
        na::dim::<P>()
    }
}

impl<N: Copy, P: Div<N, Output = P>> Div<N> for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn div(self, n: N) -> AnnotatedPoint<P> {
        AnnotatedPoint::new(self.orig1 / n, self.orig2 / n, self.point / n)
    }
}

impl<N: Copy, P: Mul<N, Output = P>> Mul<N> for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn mul(self, n: N) -> AnnotatedPoint<P> {
        AnnotatedPoint::new(self.orig1 * n, self.orig2 * n, self.point * n)
    }
}

impl<P: PartialEq> PartialEq for AnnotatedPoint<P> {
    #[inline]
    fn eq(&self, other: &AnnotatedPoint<P>) -> bool {
        self.point == other.point
    }

    #[inline]
    fn ne(&self, other: &AnnotatedPoint<P>) -> bool {
        self.point != other.point
    }
}

impl<N, P> ApproxEq<N> for AnnotatedPoint<P>
    where N: Scalar,
          P: ApproxEq<N> {
    #[inline]
    fn approx_epsilon(_: Option<AnnotatedPoint<P>>) -> N {
        ApproxEq::approx_epsilon(None::<N>)
    }

    #[inline]
    fn approx_eq_eps(&self, other: &AnnotatedPoint<P>, eps: &N) -> bool {
        self.point.approx_eq_eps(&other.point, eps)
    }

    #[inline]
    fn approx_ulps(_: Option<AnnotatedPoint<P>>) -> u32 {
        ApproxEq::approx_ulps(None::<N>)
    }

    #[inline]
    fn approx_eq_ulps(&self, other: &AnnotatedPoint<P>, ulps: u32) -> bool {
        self.point.approx_eq_ulps(&other.point, ulps)
    }
}

impl<P> Bounded for AnnotatedPoint<P> {
    fn min_value() -> AnnotatedPoint<P> {
        unimplemented!()
    }

    fn max_value() -> AnnotatedPoint<P> {
        unimplemented!()
    }
}

impl<N, P, V> NumPnt<N, V> for AnnotatedPoint<P>
    where N: Scalar,
          P: NumPnt<N, V>,
          V: Copy + NumVec<N> {
}

impl<N, P, V> FloatPnt<N, V> for AnnotatedPoint<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
}

impl<N, P, V> Point<N, V> for AnnotatedPoint<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
}
