//! Support mapping based Plane geometry.

use nalgebra::na;
use math::Vect;

/// Implicit description of a plane.
#[deriving(PartialEq, Show, Clone, Encodable, Decodable)]
pub struct Plane {
    /// The plane normal.
    normal: Vect
}

impl Plane {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub fn new(normal: Vect) -> Plane {
        unsafe { Plane::new_normalized(na::normalize(&normal)) }
    }

    /// Builds a new plane from its center and its normal.
    #[inline]
    pub unsafe fn new_normalized(normal: Vect) -> Plane {
        Plane {
            normal: normal
        }
    }
}

impl Plane {
    /// The plane normal.
    #[inline]
    pub fn normal(&self) -> Vect {
        self.normal.clone()
    }
}
