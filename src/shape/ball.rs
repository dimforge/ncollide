use na::{RealField, Unit};

use crate::math::{Isometry, Point, Vector};
use crate::shape::SupportMap;

/// A Ball shape.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Ball<N: RealField + Copy> {
    /// The radius of the ball.
    pub radius: N,
}

impl<N: RealField + Copy> Ball<N> {
    /// Creates a new ball from its radius and center.
    #[inline]
    pub fn new(radius: N) -> Ball<N> {
        Ball { radius }
    }

    /// The ball radius.
    #[inline]
    #[deprecated(note = "use the `self.radius` public field directly.")]
    pub fn radius(&self) -> N {
        self.radius
    }
}

impl<N: RealField + Copy> SupportMap<N> for Ball<N> {
    #[inline]
    fn support_point(&self, m: &Isometry<N>, dir: &Vector<N>) -> Point<N> {
        self.support_point_toward(m, &Unit::new_normalize(*dir))
    }

    #[inline]
    fn support_point_toward(&self, m: &Isometry<N>, dir: &Unit<Vector<N>>) -> Point<N> {
        Point::from(m.translation.vector) + **dir * self.radius
    }

    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        self.local_support_point_toward(&Unit::new_normalize(*dir))
    }

    #[inline]
    fn local_support_point_toward(&self, dir: &Unit<Vector<N>>) -> Point<N> {
        Point::from(**dir * self.radius)
    }
}
