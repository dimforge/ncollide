//! Support mapping based Plane shape.
use math::Vector;
use na::Unit;

/// SupportMap description of a plane.
#[derive(PartialEq, Debug, Clone)]
pub struct Plane<V> {
    /// The plane normal.
    normal: Unit<V>,
}

impl<V: Vector> Plane<V> {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub fn new(normal: Unit<V>) -> Plane<V> {
        Plane { normal: normal }
    }

    /// The plane normal.
    #[inline]
    pub fn normal(&self) -> &Unit<V> {
        &self.normal
    }
}
