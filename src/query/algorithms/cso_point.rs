use math::{Isometry, Point, Vector};
use na::{Real, Unit};
use shape::SupportMap;
use std::ops::Sub;

/// A point of a Configuration-Space Obstacle.
///
/// A Configuration-Space Obstacle (CSO) is the result of the
/// Minkowski Difference of two solids. In other words, each of its
/// points correspond to the difference of two point, each belonging
/// to a different solid.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CSOPoint<N: Real> {
    /// The point on the CSO. This is equal to `self.orig1 - self.orig2`.
    pub point: Point<N>,
    /// The original point on the first shape used to compute `self.point`.
    pub orig1: Point<N>,
    /// The original point on the second shape used to compute `self.point`.
    pub orig2: Point<N>,
}

impl<N: Real> CSOPoint<N> {
    /// Initializes a CSO point with `orig1 - orig2`.
    pub fn new(orig1: Point<N>, orig2: Point<N>) -> Self {
        let point = Point::from_coordinates(orig1 - orig2);
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

    // FIXME: in the end, we only want to keep this (i.e. remove from_shapes_toward).
    /// Same as `.from_shapes_toward` except the CSO point is computed in the local-space of `g1`.
    pub fn from_shapes_toward_local1<G1: ?Sized, G2: ?Sized>(
        g1: &G1,
        m12: &Isometry<N>,
        g2: &G2,
        dir: &Unit<Vector<N>>,
    ) -> Self
    where
        G1: SupportMap<N>,
        G2: SupportMap<N>,
    {
        let sp1 = g1.local_support_point_toward(dir);
        let sp2 = g2.support_point_toward(m12, &-*dir);

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

    /// Translate the first original point of this CSO point.
    ///
    /// This will apply the same translation to `self.point`.
    pub fn translate1(&self, dir: &Vector<N>) -> Self {
        CSOPoint::new_with_point(self.point + dir, self.orig1 + dir, self.orig2)
    }

    /// Translate the second original point of this CSO point.
    ///
    /// This will apply the opposite translation to `self.point`.
    pub fn translate2(&self, dir: &Vector<N>) -> Self {
        CSOPoint::new_with_point(self.point - dir, self.orig1, self.orig2 + dir)
    }

    /// Apply the given transformations to the second CSOPoint's original point.
    pub fn transform2(&mut self,m2: &Isometry<N>) {
        self.orig2 = m2 * self.orig2;
        self.point = self.orig1 - self.orig2.coords;
    }
}

impl<N: Real> Sub<CSOPoint<N>> for CSOPoint<N> {
    type Output = Vector<N>;

    #[inline]
    fn sub(self, rhs: CSOPoint<N>) -> Vector<N> {
        self.point - rhs.point
    }
}
