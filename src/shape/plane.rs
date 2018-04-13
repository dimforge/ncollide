//! Support mapping based Plane shape.
use na::{Unit, Real};
use math::Vector;

/// SupportMap description of a plane.
#[derive(PartialEq, Debug, Clone)]
pub struct Plane<N: Real> {
    /// The plane normal.
    normal: Unit<Vector<N>>,
}

impl<N: Real> Plane<N> {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub fn new(normal: Unit<Vector<N>>) -> Plane<N> {
        Plane { normal: normal }
    }

    /// The plane normal.
    #[inline]
    pub fn normal(&self) -> &Unit<Vector<N>> {
        &self.normal
    }
}
