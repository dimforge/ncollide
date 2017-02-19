use alga::general::Real;
use na;

use shape::SupportMap;
use math::{Point, Isometry};

/// A Ball shape.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Ball<N> {
    radius: N
}

impl<N: Real> Ball<N> {
    /// Creates a new ball from its radius and center.
    #[inline]
    pub fn new(radius: N) -> Ball<N> {
        assert!(radius > N::zero(), "A ball radius must be strictly positive.");

        Ball {
            radius: radius
        }
    }

    /// The ball radius.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for Ball<P::Real> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        m.translate_point(&P::origin()) + na::normalize(dir) * self.radius()
    }
}
