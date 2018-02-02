//! Minkowski sum.

use std::cmp::Ordering;
use std::fmt::{self, Display};
use std::ops::{Add, Sub, Mul, Div, AddAssign, MulAssign, DivAssign, Neg, Index, IndexMut};
use num::Bounded;
use approx::ApproxEq;

use alga::general::{Id, MeetSemilattice, JoinSemilattice, Lattice};
use alga::linear::{AffineSpace, EuclideanSpace};
use na::{self, Unit};
use shape::{SupportMap, Reflection};
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
#[derive(Debug)]
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
#[derive(Clone, Copy, Debug, RustcEncodable, RustcDecodable)]
pub struct AnnotatedPoint<P> {
    orig1: P,
    orig2: P,
    point: P
}

impl<P: Point> AnnotatedPoint<P> {
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
    pub fn translate_1(&mut self, t: &P::Vector) {
        self.orig1 += *t;
        self.point += *t;
    }

    #[doc(hidden)]
    #[inline]
    pub fn translate_2(&mut self, t: &P::Vector) {
        self.orig2 += *t;
        self.point += *t;
    }
}

impl<P: Point> AffineSpace for AnnotatedPoint<P> {
    type Translation = P::Vector;
}

impl<P: Point> EuclideanSpace for AnnotatedPoint<P> {
    type Coordinates = P::Vector;
    type Real        = P::Real;

    #[inline]
    fn origin() -> Self {
        Self::new(P::origin(), P::origin(), P::origin())
    }

    #[inline]
    fn scale_by(&self, s: Self::Real) -> Self {
        Self::new(
            self.point.scale_by(s),
            self.orig1.scale_by(s),
            self.orig2.scale_by(s)
        )
    }

    /// The coordinates of this point, i.e., the translation from the origin.
    #[inline]
    fn coordinates(&self) -> Self::Coordinates {
        self.point.coordinates()
    }

    /// Builds a point from its coordinates relative to the origin.
    #[inline]
    fn from_coordinates(coords: Self::Coordinates) -> Self {
        let p = P::from_coordinates(coords);
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

impl<P: Point> Index<usize> for AnnotatedPoint<P> {
    type Output = <P as Index<usize>>::Output;

    #[inline]
    fn index(&self, i: usize) -> &<P as Index<usize>>::Output {
        self.point.index(i)
    }
}

impl<P: Point> IndexMut<usize> for AnnotatedPoint<P> {
    #[inline]
    fn index_mut(&mut self, _: usize) -> &mut <P as Index<usize>>::Output {
        unimplemented!()
    }
}

impl<P: Point> Add<P::Vector> for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn add(self, other: P::Vector) -> AnnotatedPoint<P> {
        let _0_5: P::Real = na::convert(0.5f64);

        AnnotatedPoint::new(
            self.orig1 + other * _0_5,
            self.orig2 + other * _0_5,
            self.point + other
        )
    }
}

impl<P: Point> AddAssign<P::Vector> for AnnotatedPoint<P> {
    #[inline]
    fn add_assign(&mut self, other: P::Vector) {
        let _0_5: P::Real = na::convert(0.5f64);

        self.orig1 += other * _0_5;
        self.orig2 += other * _0_5;
        self.point += other;
    }
}

impl<P: Point> Sub<AnnotatedPoint<P>> for AnnotatedPoint<P> {
    type Output = P::Vector;

    #[inline]
    fn sub(self, other: AnnotatedPoint<P>) -> P::Vector {
        self.point - other.point
    }
}

impl<P: Point> Neg for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn neg(self) -> AnnotatedPoint<P> {
        AnnotatedPoint::new(-self.orig1, -self.orig2, -self.point)
    }
}

impl<P: Point> Div<P::Real> for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn div(self, n: P::Real) -> AnnotatedPoint<P> {
        AnnotatedPoint::new(self.orig1 / n, self.orig2 / n, self.point / n)
    }
}

impl<P: Point> DivAssign<P::Real> for AnnotatedPoint<P> {
    #[inline]
    fn div_assign(&mut self, n: P::Real) {
        self.orig1 /= n;
        self.orig2 /= n;
        self.point /= n;
    }
}

impl<P: Point> Mul<P::Real> for AnnotatedPoint<P> {
    type Output = AnnotatedPoint<P>;

    #[inline]
    fn mul(self, n: P::Real) -> AnnotatedPoint<P> {
        AnnotatedPoint::new(self.orig1 * n, self.orig2 * n, self.point * n)
    }
}

impl<P: Point> MulAssign<P::Real> for AnnotatedPoint<P> {
    #[inline]
    fn mul_assign(&mut self, n: P::Real) {
        self.orig1 *= n;
        self.orig2 *= n;
        self.point *= n;
    }
}

impl<P: Point> PartialOrd for AnnotatedPoint<P> {
    #[inline]
    fn partial_cmp(&self, _: &Self) -> Option<Ordering> {
        unimplemented!()
    }
}

impl<P: Point> PartialEq for AnnotatedPoint<P> {
    #[inline]
    fn eq(&self, other: &AnnotatedPoint<P>) -> bool {
        self.point == other.point
    }

    #[inline]
    fn ne(&self, other: &AnnotatedPoint<P>) -> bool {
        self.point != other.point
    }
}

impl<P: Point> MeetSemilattice for AnnotatedPoint<P> {
    #[inline]
    fn meet(&self, _: &Self) -> Self {
        unimplemented!()
    }
}

impl<P: Point> JoinSemilattice for AnnotatedPoint<P> {
    #[inline]
    fn join(&self, _: &Self) -> Self {
        unimplemented!()
    }
}


impl<P: Point> Lattice for AnnotatedPoint<P> {
    #[inline]
    fn meet_join(&self, _: &Self) -> (Self, Self) {
        unimplemented!()
    }
}

impl<P: Point> Bounded for AnnotatedPoint<P> {
    #[inline]
    fn max_value() -> Self {
        Self::new(P::max_value(), P::max_value() / na::convert(2.0), P::max_value() / na::convert(2.0))
    }

    #[inline]
    fn min_value() -> Self {
        Self::new(P::min_value(), P::min_value() / na::convert(2.0), P::min_value() / na::convert(2.0))
    }
}


impl<P> Display for AnnotatedPoint<P>
    where P: Display {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        try!(writeln!(f, "Original point 1: {}", self.orig1));
        try!(writeln!(f, "Original point 2: {}", self.orig2));
        writeln!(f, "Resulting point: {}", self.point)
    }
}

impl<P: Point> ApproxEq for AnnotatedPoint<P> {
    type Epsilon = P::Epsilon;

    #[inline]
    fn default_epsilon() -> Self::Epsilon {
        P::default_epsilon()
    }

    #[inline]
    fn default_max_relative() -> Self::Epsilon {
        P::default_max_relative()
    }

    #[inline]
    fn default_max_ulps() -> u32 {
        P::default_max_ulps()
    }

    #[inline]
    fn relative_eq(&self, other: &Self, epsilon: Self::Epsilon, max_relative: Self::Epsilon) -> bool {
        self.point.relative_eq(&other.point, epsilon, max_relative)
    }

    #[inline]
    fn ulps_eq(&self, other: &Self, epsilon: Self::Epsilon, max_ulps: u32) -> bool {
        self.point.ulps_eq(&other.point, epsilon, max_ulps)
    }
}

impl<P: Point> Point for AnnotatedPoint<P> {
    type Vector = P::Vector;

    #[inline]
    fn axpy(&mut self, a: P::Real, x: &Self, b: P::Real) {
        self.orig1.axpy(a, &x.orig1, b);
        self.orig2.axpy(a, &x.orig2, b);
        self.point.axpy(a, &x.point, b);
    }

    #[inline]
    fn ccw_face_normal(pts: &[&Self]) -> Option<Unit<P::Vector>> {
        if na::dimension::<P::Vector>() == 2 {
            P::ccw_face_normal(&[&pts[0].point, &pts[1].point])
        }
        else if na::dimension::<P::Vector>() == 3 {
            P::ccw_face_normal(&[&pts[0].point, &pts[1].point, &pts[2].point])
        }
        else {
            unimplemented!()
        }
    }
}

impl<'a, P, M, G1: ?Sized, G2: ?Sized> SupportMap<P, Id> for MinkowskiSum<'a, M, G1, G2>
    where P:  Point,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    #[inline]
    fn support_point(&self, _: &Id, dir: &P::Vector) -> P {
        self.g1().support_point(self.m1(), dir) + self.g2().support_point(self.m2(), dir).coordinates()
    }

    #[inline]
    fn support_point_toward(&self, _: &Id, dir: &Unit<P::Vector>) -> P {
        self.g1().support_point_toward(self.m1(), dir) +
        self.g2().support_point_toward(self.m2(), dir).coordinates()
    }
}

impl<'a, P, M, G1: ?Sized, G2: ?Sized>
SupportMap<AnnotatedPoint<P>, Id> for AnnotatedMinkowskiSum<'a, M, G1, G2>
    where P:  Point,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    #[inline]
    fn support_point(&self, _: &Id, dir: &P::Vector) -> AnnotatedPoint<P> {
        let orig1 = self.g1().support_point(self.m1(), dir);
        let orig2 = self.g2().support_point(self.m2(), dir);
        let point = orig1 + orig2.coordinates();

        AnnotatedPoint::new(orig1, orig2, point)
    }

    #[inline]
    fn support_point_toward(&self, _: &Id, dir: &Unit<P::Vector>) -> AnnotatedPoint<P> {
        let orig1 = self.g1().support_point_toward(self.m1(), dir);
        let orig2 = self.g2().support_point_toward(self.m2(), dir);
        let point = orig1 + orig2.coordinates();

        AnnotatedPoint::new(orig1, orig2, point)
    }
}

/// Computes the support point of the CSO `g1 - g2` on a given direction.
///
/// The result is a support point with informations about how it has been constructed.
pub fn cso_support_point<P, M, G1: ?Sized, G2: ?Sized>(m1: &M, g1: &G1,
                                                       m2: &M, g2: &G2,
                                                       dir: P::Vector)
                                                       -> AnnotatedPoint<P>
    where P:  Point,
          G1: SupportMap<P, M>,
          G2: SupportMap<P, M> {
    let rg2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &rg2);

    cso.support_point(&Id::new(), &dir)
}
