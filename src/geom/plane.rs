//! Support mapping based Plane geometry.

use nalgebra::na;
use math::Vector;

/// Implicit description of a plane.
#[deriving(Eq, Show, Clone, Encodable, Decodable)]
pub struct Plane {
    /// The plane normal.
    normal: Vector
}

impl Plane {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub fn new(normal: Vector) -> Plane {
        unsafe { Plane::new_normalized(na::normalize(&normal)) }
    }

    /// Builds a new plane from its center and its normal.
    #[inline]
    pub unsafe fn new_normalized(normal: Vector) -> Plane {
        Plane {
            normal: normal
        }
    }
}

impl Plane {
    /// The plane normal.
    #[inline]
    pub fn normal(&self) -> Vector {
        self.normal.clone()
    }
}
