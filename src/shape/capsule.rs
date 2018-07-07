//! Support mapping based Capsule shape.

use na::{self, Unit, Real};

use utils::IsometryOps;
use shape::SupportMap;
use math::{Isometry, Point, Vector};

/// SupportMap description of a capsule shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone)]
pub struct Capsule<N> {
    half_height: N,
    radius: N,
}

impl<N: Real> Capsule<N> {
    /// Creates a new capsule.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the capsule along the `y` axis.
    /// * `radius` - radius of the rounded part of the capsule.
    pub fn new(half_height: N, radius: N) -> Capsule<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Capsule {
            half_height: half_height,
            radius: radius,
        }
    }

    /// The capsule half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height
    }

    /// The radius of the capsule's rounded part.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius
    }
}

impl<N: Real> SupportMap<N> for Capsule<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        self.local_support_point_toward(&Unit::new_normalize(*dir))
    }

    #[inline]
    fn local_support_point_toward(&self, dir: &Unit<Vector<N>>) -> Point<N> {
        let mut res: Vector<N> = na::zero();

        if dir[1].is_negative() {
            res[1] = -self.half_height()
        } else {
            res[1] = self.half_height()
        }

        Point::from_coordinates(res + dir.as_ref() * self.radius())
    }
}
