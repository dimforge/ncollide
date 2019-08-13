//! Support mapping based Plane shape.
use crate::math::Vector;
use na::{RealField, Unit};

/// SupportMap description of a plane.
#[derive(PartialEq, Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Plane<N: RealField> {
    /// The plane normal.
    normal: Unit<Vector<N>>,
}

impl<N: RealField> Plane<N> {
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
