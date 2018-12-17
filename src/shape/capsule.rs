//! Support mapping based Capsule shape.

use na::{self, Unit, Real};

use utils::IsometryOps;
use shape::SupportMap;
use math::{Isometry, Point, Vector};

/// SupportMap description of a capsule shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    fn support_point(&self, m: &Isometry<N>, dir: &Vector<N>) -> Point<N> {
        self.support_point_toward(m, &Unit::new_normalize(*dir))
    }

    #[inline]
    fn support_point_toward(&self, m: &Isometry<N>, dir: &Unit<Vector<N>>) -> Point<N> {
        let local_dir = m.inverse_transform_vector(dir);

        let mut res: Vector<N> = na::zero();

        if local_dir[1].is_negative() {
            res[1] = -self.half_height()
        } else {
            res[1] = self.half_height()
        }

        m * Point::from_coordinates(res + local_dir * self.radius())
    }
}
