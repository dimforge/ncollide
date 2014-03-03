//!
//! Support mapping based Cylinder geometry.
//!

use nalgebra::na::Cast;
use math::Scalar;

/// Implicit description of a cylinder geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, Show, Clone, Encodable, Decodable)]
pub struct Cylinder {
    priv half_height: Scalar,
    priv radius:      Scalar,
    priv margin:      Scalar
}

impl Cylinder {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `x` axis.
    /// * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: Scalar, radius: Scalar) -> Cylinder {
        Cylinder::new_with_margin(half_height, radius, Cast::from(0.04))
    }

    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `x` axis.
    /// * `radius` - the length of the cylinder along all other axis.
    pub fn new_with_margin(half_height: Scalar, radius: Scalar, margin: Scalar) -> Cylinder {
        assert!(half_height.is_positive() && radius.is_positive());

        Cylinder {
            half_height: half_height,
            radius:      radius,
            margin:      margin
        }
    }
}

impl Cylinder {
    /// The cylinder half length along the `x` axis.
    pub fn half_height(&self) -> Scalar {
        self.half_height.clone()
    }

    /// The radius of the cylinder along all but the `x` axis.
    pub fn radius(&self) -> Scalar {
        self.radius.clone()
    }

    /// Size of the margin around the cylinder.
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }
}
