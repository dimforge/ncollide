use na;
use math::Scalar;

/// A Ball shape.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Ball<N> {
    radius: N
}

impl<N: Scalar> Ball<N> {
    /// Creates a new ball from its radius and center.
    #[inline]
    pub fn new(radius: N) -> Ball<N> {
        assert!(radius > na::zero(), "A ball radius must be strictly positive.");

        Ball {
            radius: radius
        }
    }

    /// The ball radius.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}
