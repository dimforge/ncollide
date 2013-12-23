//!
//! Support mapping based Capsule geometry.
//!  

/// Implicit description of a capsule geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, ToStr, Clone, Encodable, Decodable)]
pub struct Capsule<N> {
    priv half_height: N,
    priv radius:      N,
    priv margin:      N
}

impl<N: Signed> Capsule<N> {
    /// Creates a new capsule.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the capsule along the `x` axis.
    ///     * `radius` - radius of the rounded part of the capsule.
    pub fn new(half_height: N, radius: N, margin: N) -> Capsule<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Capsule {
            half_height: half_height,
            radius:      radius,
            margin:      margin
        }
    }
}

impl<N: Clone> Capsule<N> {
    /// The capsule half length along the `x` axis.
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the capsule's rounded part.
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}
