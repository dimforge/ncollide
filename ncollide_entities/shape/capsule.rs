//! Support mapping based Capsule shape.

use math::Scalar;

/// SupportMap description of a capsule shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Capsule<N> {
    half_height: N,
    radius:      N,
}

impl<N> Capsule<N>
    where N: Scalar {
    /// Creates a new capsule.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the capsule along the `y` axis.
    /// * `radius` - radius of the rounded part of the capsule.
    pub fn new(half_height: N, radius: N) -> Capsule<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Capsule {
            half_height: half_height,
            radius:      radius,
        }
    }

    /// The capsule half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the capsule's rounded part.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}
