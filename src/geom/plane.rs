//! Support mapping based Plane geometry.

use nalgebra::na;
use math::V;

/// Implicit description of a plane.
#[deriving(Eq, Show, Clone, Encodable, Decodable)]
pub struct Plane {
    /// The plane normal.
    normal: V
}

impl Plane {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub fn new(normal: V) -> Plane {
        unsafe { Plane::new_normalized(na::normalize(&normal)) }
    }

    /// Builds a new plane from its center and its normal.
    #[inline]
    pub unsafe fn new_normalized(normal: V) -> Plane {
        Plane {
            normal: normal
        }
    }
}

impl Plane {
    /// The plane normal.
    #[inline]
    pub fn normal(&self) -> V {
        self.normal.clone()
    }
}
