use std::ops::Sub;
use na::{Real, Unit};
use shape::SupportMap;
use math::{Isometry, Point, Vector};

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CSOPoint<N: Real> {
    pub point: Point<N>,
    pub orig1: Point<N>,
    pub orig2: Point<N>,
}

impl<N: Real> CSOPoint<N> {
    pub fn new(orig1: Point<N>, orig2: Point<N>) -> Self {
        let point = Point::from_coordinates(orig1 - orig2);
        Self::new_with_point(point, orig1, orig2)
    }

    pub fn new_with_point(point: Point<N>, orig1: Point<N>, orig2: Point<N>) -> Self {
        CSOPoint {
            point,
            orig1,
            orig2,
        }
    }

    pub fn single_point(point: Point<N>) -> Self {
        Self::new_with_point(point, point, Point::origin())
    }

    pub fn origin() -> Self {
        CSOPoint::new(Point::origin(), Point::origin())
    }

    pub fn from_shapes<G1: ?Sized, G2: ?Sized>(
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
        let sp1 = g1.support_point(m1, dir);
        let sp2 = g2.support_point(m2, &-*dir);

        CSOPoint::new(sp1, sp2)
    }
}

impl<N: Real> Sub<CSOPoint<N>> for CSOPoint<N> {
    type Output = Vector<N>;

    #[inline]
    fn sub(self, rhs: CSOPoint<N>) -> Vector<N> {
        self.point - rhs.point
    }
}
