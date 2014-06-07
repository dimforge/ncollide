//! Support mapping based Capsule geometry.

use math::Scalar;

/// Implicit description of a capsule geometry with its principal axis aligned with the `y` axis.
#[deriving(PartialEq, Show, Clone, Encodable, Decodable)]
pub struct Capsule {
    half_height: Scalar,
    radius:      Scalar,
}

impl Capsule {
    /// Creates a new capsule.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the capsule along the `y` axis.
    /// * `radius` - radius of the rounded part of the capsule.
    pub fn new(half_height: Scalar, radius: Scalar) -> Capsule {
        assert!(half_height.is_positive() && radius.is_positive());

        Capsule {
            half_height: half_height,
            radius:      radius,
        }
    }
}

impl Capsule {
    /// The capsule half length along the `y` axis.
    pub fn half_height(&self) -> Scalar {
        self.half_height.clone()
    }

    /// The radius of the capsule's rounded part.
    pub fn radius(&self) -> Scalar {
        self.radius.clone()
    }
}
