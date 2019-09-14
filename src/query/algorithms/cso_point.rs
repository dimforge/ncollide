use crate::math::{Isometry, Point, Vector};
use crate::shape::SupportMap;
use na::{RealField, Unit};
use std::ops::Sub;

/// A point of a Configuration-Space Obstacle.
///
/// A Configuration-Space Obstacle (CSO) is the result of the
/// Minkowski Difference of two solids. In other words, each of its
/// points correspond to the difference of two point, each belonging
/// to a different solid.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CSOPoint<N: RealField> {
    /// The point on the CSO. This is equal to `self.orig1 - self.orig2`, unless this CSOPoint
    /// has been translated with self.translate.
    pub point: Point<N>,
    /// The original point on the first shape used to compute `self.point`.
    pub orig1: Point<N>,
    /// The original point on the second shape used to compute `self.point`.
    pub orig2: Point<N>,
}

impl<N: RealField> CSOPoint<N> {
    /// Initializes a CSO point with `orig1 - orig2`.
    pub fn new(orig1: Point<N>, orig2: Point<N>) -> Self {
        let point = Point::from(orig1 - orig2);
        Self::new_with_point(point, orig1, orig2)
    }

    /// Initializes a CSO point with all information provided.
    ///
    /// It is assumed, but not checked, that `point == orig1 - orig2`.
    pub fn new_with_point(point: Point<N>, orig1: Point<N>, orig2: Point<N>) -> Self {
        CSOPoint {
            point,
            orig1,
            orig2,
        }
    }

    /// Initializes a CSO point where both original points are equal.
    pub fn single_point(point: Point<N>) -> Self {
        Self::new_with_point(point, point, Point::origin())
    }

    /// CSO point where all components are set to zero.
    pub fn origin() -> Self {
        CSOPoint::new(Point::origin(), Point::origin())
    }

    /// Computes the support point of the CSO of `g1` and `g2` toward the unit direction `dir`.
    pub fn from_shapes_toward<G1: ?Sized, G2: ?Sized>(
        m1: &Isometry<N>,
        g1: &G1,
        m2: &Isometry<N>,
        g2: &G2,
        dir: &Unit<Vector<N>>,
    ) -> Self
    where
        G1: SupportMap<N>,
        G2: SupportMap<N>,
    {
        let sp1 = g1.support_point_toward(m1, dir);
        let sp2 = g2.support_point_toward(m2, &-*dir);

        CSOPoint::new(sp1, sp2)
    }

    /// Computes the support point of the CSO of `g1` and `g2` toward the direction `dir`.
    pub fn from_shapes<G1: ?Sized, G2: ?Sized>(
        m1: &Isometry<N>,
        g1: &G1,
        m2: &Isometry<N>,
        g2: &G2,
        dir: &Vector<N>,
    ) -> Self
    where
        G1: SupportMap<N>,
        G2: SupportMap<N>,
    {
        let sp1 = g1.support_point(m1, dir);
        let sp2 = g2.support_point(m2, &-*dir);

        CSOPoint::new(sp1, sp2)
    }

    /// Translate the CSO point.
    pub fn translate(&self, dir: &Vector<N>) -> Self {
        CSOPoint::new_with_point(self.point + dir, self.orig1, self.orig2)
    }

    /// Translate in-place the CSO point.
    pub fn translate_mut(&mut self, dir: &Vector<N>) {
        self.point += dir;
    }
}

impl<N: RealField> Sub<CSOPoint<N>> for CSOPoint<N> {
    type Output = Vector<N>;

    #[inline]
    fn sub(self, rhs: CSOPoint<N>) -> Vector<N> {
        self.point - rhs.point
    }
}
