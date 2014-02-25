//!
//! Support mapping based Ball geometry.
//!

use math::N;

/// A Ball geometry.
#[deriving(Eq, Show, Clone, Encodable, Decodable)]
pub struct Ball {
    priv radius: N
}

impl Ball {
    /// Creates a new ball from its radius and center.
    #[inline]
    pub fn new(radius: N) -> Ball {
        Ball { radius: radius }
    }
}

impl Ball {
    /// The ball radius.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}
