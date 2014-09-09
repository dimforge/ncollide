//!
//! Support mapping based Cylinder geometry.
//!

use nalgebra::na;
use math::Scalar;

/// Implicit description of a cylinder geometry with its principal axis aligned with the `y` axis.
#[deriving(PartialEq, Show, Clone, Encodable, Decodable)]
pub struct Cylinder {
    half_height: Scalar,
    radius:      Scalar,
}

impl Cylinder {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `y` axis.
    /// * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: Scalar, radius: Scalar) -> Cylinder {
        assert!(half_height.is_positive() && radius.is_positive());

        Cylinder {
            half_height: half_height,
            radius:      radius
        }
    }
}

impl Cylinder {
    /// The cylinder half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> Scalar {
        self.half_height.clone()
    }

    /// The radius of the cylinder along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> Scalar {
        self.radius.clone()
    }
}
