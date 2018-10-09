use na::{Real, Unit};

use math::{Isometry, Point, Vector};
use shape::SupportMap;

/// A Ball shape.
#[derive(PartialEq, Debug, Clone)]
pub struct Ball<N: Real> {
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

        Ball { radius }
    }

    /// The ball radius.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius
    }
}

impl<N: Real> SupportMap<N> for Ball<N> {
    #[inline]
    fn support_point(&self, m: &Isometry<N>, dir: &Vector<N>) -> Point<N> {
        self.support_point_toward(m, &Unit::new_normalize(*dir))
    }

    #[inline]
    fn support_point_toward(&self, m: &Isometry<N>, dir: &Unit<Vector<N>>) -> Point<N> {
        m * Point::origin() + **dir * self.radius()
    }
}
