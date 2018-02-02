use alga::general::Real;
use na::Unit;

use shape::SupportMap;
use math::{Isometry, Point};

/// A Ball shape.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Ball<N> {
    radius: N,
}

impl<N: Real> Ball<N> {
    /// Creates a new ball from its radius and center.
    #[inline]
    pub fn new(radius: N) -> Ball<N> {
        assert!(
            radius > N::zero(),
            "A ball radius must be strictly positive."
        );

        Ball { radius: radius }
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
        self.support_point_toward(m, &Unit::new_normalize(*dir))
    }

    #[inline]
    fn support_point_toward(&self, m: &M, dir: &Unit<P::Vector>) -> P {
        m.translate_point(&P::origin()) + **dir * self.radius()
    }
}
