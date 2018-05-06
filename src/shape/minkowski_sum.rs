//! Minkowski sum.

use std::cmp::Ordering;
use std::fmt::{self, Display};
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub};
use num::Bounded;
use approx::ApproxEq;

use alga::general::{Id, JoinSemilattice, Lattice, MeetSemilattice};
use alga::linear::{AffineSpace, EuclideanSpace};
use na::{self, Unit};
use shape::{Reflection, SupportMap};
use math::Point;

/// Type of an implicit representation of the Configuration Space Obstacle
/// formed by two geometric objects.
pub type CSO<'a, M, G1, G2> = MinkowskiSum<'a, M, G1, Reflection<'a, G2>>;

/// Type of an implicit representation of the Configuration Space Obstacle
/// formed by two geometric objects. Uses return `AnnotatedPoint` instead of regular points, i.e.,
/// keeps tracks of the original points that yielded a given CSO support point.
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
#[derive(Debug)]
pub struct MinkowskiSum<'a, M: 'a, G1: ?Sized + 'a, G2: ?Sized + 'a> {
    m1: &'a M,
    g1: &'a G1,
    m2: &'a M,
    g2: &'a G2,
}

impl<'a, M, G1: ?Sized, G2: ?Sized> MinkowskiSum<'a, M, G1, G2> {
    /**
     * Builds the Minkowski sum of two shapes. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(m1: &'a M, g1: &'a G1, m2: &'a M, g2: &'a G2) -> MinkowskiSum<'a, M, G1, G2> {
        MinkowskiSum {
            m1: m1,
            g1: g1,
            m2: m2,
            g2: g2,
        }
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
#[derive(Debug)]
pub struct AnnotatedMinkowskiSum<'a, M: 'a, G1: ?Sized + 'a, G2: ?Sized + 'a> {
    m1: &'a M,
    g1: &'a G1,
    m2: &'a M,
    g2: &'a G2,
}

impl<'a, M, G1: ?Sized, G2: ?Sized> AnnotatedMinkowskiSum<'a, M, G1, G2> {
    /**
     * Builds the Minkowski sum of two shapes. Since the representation is
     * implicit, this is done in constant time.
     */
    #[inline]
    pub fn new(
        m1: &'a M,
        g1: &'a G1,
        m2: &'a M,
        g2: &'a G2,
    ) -> AnnotatedMinkowskiSum<'a, M, G1, G2> {
        AnnotatedMinkowskiSum {
            m1: m1,
            g1: g1,
            m2: m2,
            g2: g2,
        }
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
#[derive(Clone, Copy, Debug)]
pub struct AnnotatedPoint<P> {
    orig1: Point<N>,
    orig2: Point<N>,
    point: Point<N>,
}

impl<N: Real> AnnotatedPoint<P> {
    #[doc(hidden)]
    #[inline]
    pub fn new(orig1: Point<N>, orig2: Point<N>, point: Point<N>) -> AnnotatedPoint<P> {
        AnnotatedPoint {
            orig1: orig1,
            orig2: orig2,
            point: point,
        }
    }

    #[doc(hidden)]
    #[inline]
    pub fn point<'r>(&'r self) -> &'r P {
        &self.point
    }

    #[doc(hidden)]
    #[inline]
    pub fn orig1(&self) -> &Point<N> {
        &self.orig1
    }

    #[doc(hidden)]
    #[inline]
    pub fn orig2(&self) -> &Point<N> {
        &self.orig2
    }

    #[doc(hidden)]
    #[inline]
    pub fn translate_1(&mut self, t: &Vector<N>) {
        self.orig1 += *t;
        self.point += *t;
    }

    #[doc(hidden)]
    #[inline]
    pub fn translate_2(&mut self, t: &Vector<N>) {
        self.orig2 += *t;
        self.point += *t;
    }
}

impl<N: Real> AffineSpace for AnnotatedPoint<P> {
    type Translation = Vector<N>;
}

impl<N: Real> EuclideanSpace for AnnotatedPoint<P> {
    type Coordinates = Vector<N>;
    type Real = N;

    #[inline]
    fn origin() -> Self {
        Self::new(Point::origin(), Point::origin(), Point::origin())
    }

    #[inline]
    fn scale_by(&self, s: Self::Real) -> Self {
        Self::new(
            self.point.scale_by(s),
            self.orig1.scale_by(s),
            self.orig2.scale_by(s),
        )
    }

    /// The coordinates of this point, i.e., the translation from the origin.
    #[inline]
    fn coordinates(&self) -> Self::Coordinates {
        self.point.coords
    }

    /// Builds a point from its coordinates relative to the origin.
    #[inline]
    fn from_coordinates(coords: Self::Coordinates) -> Self {
        let p = Point::from_coordinates(coords);
        Self::new(p, p, p)
    }

    /// The distance between two points.
    #[inline]
    fn distance_squared(&self, b: &Self) -> Self::Real {
        self.point.distance_squared(&b.point)
    }

    /// The distance between two points.
    #[inline]
    fn distance(&self, b: &Self) -> Self::Real {
        self.point.distance(&b.point)
    }
}

impl<N: Real> Index<usize> for AnnotatedPoint<P> {
    type Output = <P as Index<usize>>::Output;

    #[inline]
    fn index(&self, i: usize) -> &<P as Index<usize>>::Output {
        self.point.index(i)
    }
}

impl<N: Real> IndexMut<usize> for AnnotatedPoint<P> {
    #[inline]
    fn index_mut(&mut self, _: usize) -> &mut <P as Index<usize>>::Output {
        unimplemented!()
    }
}

impl<N: Real> Add<Vector<N>> for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn add(self, other: Vector<N>) -> AnnotatedPoint<P> {
        let _0_5: N = na::convert(0.5f64);

        AnnotatedPoint::new(
            self.orig1 + other * _0_5,
            self.orig2 + other * _0_5,
            self.point + other,
        )
    }
}

impl<N: Real> AddAssign<Vector<N>> for AnnotatedPoint<P> {
    #[inline]
    fn add_assign(&mut self, other: Vector<N>) {
        let _0_5: N = na::convert(0.5f64);

        self.orig1 += other * _0_5;
        self.orig2 += other * _0_5;
        self.point += other;
    }
}

impl<N: Real> Sub<AnnotatedPoint<P>> for AnnotatedPoint<P> {
    type Output = Vector<N>;

    #[inline]
    fn sub(self, other: AnnotatedPoint<P>) -> Vector<N> {
        self.point - other.point
    }
}

impl<N: Real> Neg for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn neg(self) -> AnnotatedPoint<P> {
        AnnotatedPoint::new(-self.orig1, -self.orig2, -self.point)
    }
}

impl<N: Real> Div<N> for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn div(self, n: N) -> AnnotatedPoint<P> {
        AnnotatedPoint::new(self.orig1 / n, self.orig2 / n, self.point / n)
    }
}

impl<N: Real> DivAssign<N> for AnnotatedPoint<P> {
    #[inline]
    fn div_assign(&mut self, n: N) {
        self.orig1 /= n;
        self.orig2 /= n;
        self.point /= n;
    }
}

impl<N: Real> Mul<N> for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn mul(self, n: N) -> AnnotatedPoint<P> {
        AnnotatedPoint::new(self.orig1 * n, self.orig2 * n, self.point * n)
    }
}

impl<N: Real> MulAssign<N> for AnnotatedPoint<P> {
    #[inline]
    fn mul_assign(&mut self, n: N) {
        self.orig1 *= n;
        self.orig2 *= n;
        self.point *= n;
    }
}

impl<N: Real> PartialOrd for AnnotatedPoint<P> {
    #[inline]
    fn partial_cmp(&self, _: &Self) -> Option<Ordering> {
        unimplemented!()
    }
}

impl<N: Real> PartialEq for AnnotatedPoint<P> {
    #[inline]
    fn eq(&self, other: &AnnotatedPoint<P>) -> bool {
        self.point == other.point
    }

    #[inline]
    fn ne(&self, other: &AnnotatedPoint<P>) -> bool {
        self.point != other.point
    }
}

impl<N: Real> MeetSemilattice for AnnotatedPoint<P> {
    #[inline]
    fn meet(&self, _: &Self) -> Self {
        unimplemented!()
    }
}

impl<N: Real> JoinSemilattice for AnnotatedPoint<P> {
    #[inline]
    fn join(&self, _: &Self) -> Self {
        unimplemented!()
    }
}

impl<N: Real> Lattice for AnnotatedPoint<P> {
    #[inline]
    fn meet_join(&self, _: &Self) -> (Self, Self) {
        unimplemented!()
    }
}

impl<N: Real> Bounded for AnnotatedPoint<P> {
    #[inline]
    fn max_value() -> Self {
        Self::new(
            Point::max_value(),
            Point::max_value() / na::convert(2.0),
            Point::max_value() / na::convert(2.0),
        )
    }

    #[inline]
    fn min_value() -> Self {
        Self::new(
            Point::min_value(),
            Point::min_value() / na::convert(2.0),
            Point::min_value() / na::convert(2.0),
        )
    }
}

impl<P> Display for AnnotatedPoint<P>
where
    P: Display,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        try!(writeln!(f, "Original point 1: {}", self.orig1));
        try!(writeln!(f, "Original point 2: {}", self.orig2));
        writeln!(f, "Resulting point: {}", self.point)
    }
}

impl<N: Real> ApproxEq for AnnotatedPoint<P> {
    type Epsilon = Point::Epsilon;

    #[inline]
    fn default_epsilon() -> Self::Epsilon {
        Point::default_epsilon()
    }

    #[inline]
    fn default_max_relative() -> Self::Epsilon {
        Point::default_max_relative()
    }

    #[inline]
    fn default_max_ulps() -> u32 {
        Point::default_max_ulps()
    }

    #[inline]
    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        self.point.relative_eq(&other.point, epsilon, max_relative)
    }

    #[inline]
    fn ulps_eq(&self, other: &Self, epsilon: Self::Epsilon, max_ulps: u32) -> bool {
        self.point.ulps_eq(&other.point, epsilon, max_ulps)
    }
}

impl<N: Real> Point for AnnotatedPoint<P> {
    type Vector = Vector<N>;

    #[inline]
    fn axpy(&mut self, a: N, x: &Self, b: N) {
        self.orig1.axpy(a, &x.orig1, b);
        self.orig2.axpy(a, &x.orig2, b);
        self.point.axpy(a, &x.point, b);
    }

    #[inline]
    fn ccw_face_normal(pts: &[&Self]) -> Option<Unit<Vector<N>>> {
        if na::dimension::<Vector<N>>() == 2 {
            Point::ccw_face_normal(&[&pts[0].point, &pts[1].point])
        } else if na::dimension::<Vector<N>>() == 3 {
            Point::ccw_face_normal(&[&pts[0].point, &pts[1].point, &pts[2].point])
        } else {
            unimplemented!()
        }
    }
}

impl<'a, N, G1: ?Sized, G2: ?Sized> SupportMap<P, Id> for MinkowskiSum<'a, M, G1, G2>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    #[inline]
    fn support_point(&self, _: &Id, dir: &Vector<N>) -> Point<N> {
        self.g1().support_point(self.m1(), dir)
            + self.g2().support_point(self.m2(), dir).coords
    }

    #[inline]
    fn support_point_toward(&self, _: &Id, dir: &Unit<Vector<N>>) -> Point<N> {
        self.g1().support_point_toward(self.m1(), dir)
            + self.g2().support_point_toward(self.m2(), dir).coords
    }
}

impl<'a, N, G1: ?Sized, G2: ?Sized> SupportMap<AnnotatedPoint<P>, Id>
    for AnnotatedMinkowskiSum<'a, M, G1, G2>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    #[inline]
    fn support_point(&self, _: &Id, dir: &Vector<N>) -> AnnotatedPoint<P> {
        let orig1 = self.g1().support_point(self.m1(), dir);
        let orig2 = self.g2().support_point(self.m2(), dir);
        let point = orig1 + orig2.coords;

        AnnotatedPoint::new(orig1, orig2, point)
    }

    #[inline]
    fn support_point_toward(&self, _: &Id, dir: &Unit<Vector<N>>) -> AnnotatedPoint<P> {
        let orig1 = self.g1().support_point_toward(self.m1(), dir);
        let orig2 = self.g2().support_point_toward(self.m2(), dir);
        let point = orig1 + orig2.coords;

        AnnotatedPoint::new(orig1, orig2, point)
    }
}

/// Computes the support point of the CSO `g1 - g2` on a given direction.
///
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    dir: Vector<N>,
) -> AnnotatedPoint<P>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &rg2);

    cso.support_point(&Isometry::identity(), &dir)
}
