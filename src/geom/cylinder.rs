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
    margin:      Scalar
}

impl Cylinder {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `y` axis.
    /// * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: Scalar, radius: Scalar) -> Cylinder {
        Cylinder::new_with_margin(half_height, radius, na::cast(0.04f64))
    }

    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `y` axis.
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

    /// Size of the margin around the cylinder.
    #[inline]
    pub fn margin(&self) -> Scalar {
        self.margin.clone()
    }
}
