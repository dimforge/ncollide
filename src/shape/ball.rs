use na::{Real, Unit};

use shape::SupportMap;
use math::{Vector, Point, Isometry};

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

    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        self.local_support_point_toward(&Unit::new_normalize(*dir))
    }

    #[inline]
    fn local_support_point_toward(&self, dir: &Unit<Vector<N>>) -> Point<N> {
        Point::from_coordinates(**dir * self.radius())
    }
}
