//! Support mapping based Cone geometry.

use nalgebra::na::Cast;
use math::Scalar;

/// Implicit description of a cylinder geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, Show, Clone, Encodable, Decodable)]
pub struct Cone {
    priv half_height: Scalar,
    priv radius:      Scalar,
    priv margin:      Scalar
}

impl Cone {
    /// Creates a new cone.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cone along the `x` axis.
    /// * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: Scalar, radius: Scalar) -> Cone {
        Cone::new_with_margin(half_height, radius, Cast::from(0.04))
    }

    /// Creates a new cone with a custom margin.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cone along the `x` axis.
    /// * `radius` - the length of the cone along all other axis.
    /// * `margin` - the  cone margin.
    pub fn new_with_margin(half_height: Scalar, radius: Scalar, margin: Scalar) -> Cone {
        assert!(half_height.is_positive() && radius.is_positive());

        Cone {
            half_height: half_height,
            radius:      radius,
            margin:      margin
        }
    }
}

impl Cone {
    /// The cone half length along the `x` axis.
    pub fn half_height(&self) -> Scalar {
        self.half_height.clone()
    }

    /// The radius of the cone along all but the `x` axis.
    pub fn radius(&self) -> Scalar {
        self.radius.clone()
    }

    /// The margin around the cone.
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }
}
