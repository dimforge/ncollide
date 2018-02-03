//! Support mapping based Capsule shape.

use num::Signed;

use alga::general::Real;
use na::{self, Unit};

use shape::SupportMap;
use math::{Isometry, Point};

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

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for Capsule<P::Real> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        self.support_point_toward(m, &Unit::new_normalize(*dir))
    }

    #[inline]
    fn support_point_toward(&self, m: &M, dir: &Unit<P::Vector>) -> P {
        let local_dir = m.inverse_rotate_vector(dir);

        let mut res: P::Vector = na::zero();

        if local_dir[1].is_negative() {
            res[1] = -self.half_height()
        } else {
            res[1] = self.half_height()
        }

        m.transform_point(&(P::from_coordinates(res + local_dir * self.radius())))
    }
}
