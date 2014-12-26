//! Support mapping based Plane shape.

use na;
use na::Norm;
use math::Scalar;

/// SupportMap description of a plane.
#[deriving(PartialEq, Show, Clone, RustcEncodable, RustcDecodable)]
pub struct Plane<V> {
    /// The plane normal.
    normal: V
}

impl<N: Scalar, V: Norm<N>> Plane<V> {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub fn new(normal: V) -> Plane<V> {
        unsafe { Plane::new_normalized(na::normalize(&normal)) }
    }
}

impl<V> Plane<V> {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub unsafe fn new_normalized(normal: V) -> Plane<V> {
        Plane {
            normal: normal
        }
    }

    /// The plane normal.
    #[inline]
    pub fn normal(&self) -> &V {
        &self.normal
    }
}
