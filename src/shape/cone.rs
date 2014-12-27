//! Support mapping based Cone shape.

use math::Scalar;

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[deriving(PartialEq, Show, Clone, RustcEncodable, RustcDecodable)]
pub struct Cone<N> {
    half_height: N,
    radius:      N,
}

impl<N> Cone<N>
    where N: Scalar {
    /// Creates a new cone.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cone along the `y` axis.
    /// * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: N, radius: N) -> Cone<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cone {
            half_height: half_height,
            radius:      radius
        }
    }

    /// The cone half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the cone along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}
