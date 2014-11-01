//!
//! Support mapping based Cylinder shape.
//!

use math::Scalar;

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[deriving(PartialEq, Show, Clone, Encodable, Decodable)]
pub struct Cylinder<N> {
    half_height: N,
    radius:      N,
}

impl<N> Cylinder<N>
    where N: Scalar {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `y` axis.
    /// * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: N, radius: N) -> Cylinder<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cylinder {
            half_height: half_height,
            radius:      radius
        }
    }

    /// The cylinder half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the cylinder along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}
