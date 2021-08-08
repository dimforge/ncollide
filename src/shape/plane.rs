//! Support mapping based Plane shape.
use crate::math::Vector;
use na::{RealField, Unit};

/// SupportMap description of a plane.
#[derive(PartialEq, Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Plane<N: RealField + Copy> {
    /// The plane normal.
    pub normal: Unit<Vector<N>>,
}

impl<N: RealField + Copy> Plane<N> {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub fn new(normal: Unit<Vector<N>>) -> Plane<N> {
        Plane { normal }
    }

    /// The plane normal.
    #[inline]
    #[deprecated(note = "use the `self.normal` public field directly.")]
    pub fn normal(&self) -> &Unit<Vector<N>> {
        &self.normal
    }
}
