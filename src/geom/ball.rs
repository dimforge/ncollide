//!
//! Support mapping based Ball geometry.
//!

/**
 * A Ball geometry.
 * 
 *  - `N`: numeric type used for the ball radius.
 */
#[deriving(Eq, ToStr, Clone, Encodable, Decodable)]
pub struct Ball<N> {
    priv radius: N
}

impl<N> Ball<N> {
    /// Creates a new ball from its radius and center.
    #[inline]
    pub fn new(radius: N) -> Ball<N> {
        Ball { radius: radius }
    }
}

impl<N: Clone> Ball<N> {
    /// The ball radius.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}
