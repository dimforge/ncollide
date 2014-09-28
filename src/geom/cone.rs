//! Support mapping based Cone geometry.

use na;
use math::Scalar;

/// Implicit description of a cylinder geometry with its principal axis aligned with the `y` axis.
#[deriving(PartialEq, Show, Clone, Encodable, Decodable)]
pub struct Cone {
    half_height: Scalar,
    radius:      Scalar,
}

impl Cone {
    /// Creates a new cone.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cone along the `y` axis.
    /// * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: Scalar, radius: Scalar) -> Cone {
        assert!(half_height.is_positive() && radius.is_positive());

        Cone {
            half_height: half_height,
            radius:      radius
        }
    }
}

impl Cone {
    /// The cone half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> Scalar {
        self.half_height.clone()
    }

    /// The radius of the cone along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> Scalar {
        self.radius.clone()
    }
}
