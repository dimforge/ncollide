use na::{RealField, Unit};

use crate::math::{Isometry, Point, Vector};
use crate::shape::SupportMap;

/// A support mapping that is always equal to the origin, independently from its transform.
pub struct ConstantOrigin;

impl<N: RealField> SupportMap<N> for ConstantOrigin {
    #[inline]
    fn support_point(&self, _: &Isometry<N>, _: &Vector<N>) -> Point<N> {
        Point::origin()
    }

    #[inline]
    fn support_point_toward(&self, _: &Isometry<N>, _: &Unit<Vector<N>>) -> Point<N> {
        Point::origin()
    }
}

/// The Minkowski sum of a shape and a ball.
pub struct DilatedShape<'a, N: RealField, S: ?Sized + SupportMap<N>> {
    /// The shape involved in the Minkowski sum.
    pub shape: &'a S,
    /// The radius of the ball involved in the Minkoski sum.
    pub radius: N,
}

impl<'a, N: RealField, S: ?Sized + SupportMap<N>> SupportMap<N> for DilatedShape<'a, N, S> {
    #[inline]
    fn support_point(&self, m: &Isometry<N>, dir: &Vector<N>) -> Point<N> {
        self.support_point_toward(m, &Unit::new_normalize(*dir))
    }

    #[inline]
    fn support_point_toward(&self, m: &Isometry<N>, dir: &Unit<Vector<N>>) -> Point<N> {
        self.shape.support_point_toward(m, dir) + **dir * self.radius
    }
}
