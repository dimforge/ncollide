//! Support mapping based Cylinder shape.

use crate::math::{Point, Vector};
use crate::shape::SupportMap;
use na::{self, RealField};

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Cylinder<N> {
    /// The half-height of the cylinder.
    pub half_height: N,
    /// The radius fo the cylinder.
    pub radius: N,
}

impl<N: RealField> Cylinder<N> {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `y` axis.
    /// * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: N, radius: N) -> Cylinder<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cylinder {
            half_height,
            radius,
        }
    }

    /// The cylinder half length along the `y` axis.
    #[inline]
    #[deprecated(note = "use the `self.half_height` field directly.")]
    pub fn half_height(&self) -> N {
        self.half_height
    }

    /// The radius of the cylinder along all but the `y` axis.
    #[inline]
    #[deprecated(note = "use the `self.radius` field directly.")]
    pub fn radius(&self) -> N {
        self.radius
    }
}

impl<N: RealField> SupportMap<N> for Cylinder<N> {
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        let mut vres = *dir;

        vres[1] = na::zero();

        if vres.normalize_mut().is_zero() {
            vres = na::zero()
        } else {
            vres = vres * self.radius;
        }

        vres[1] = dir[1].copysign(self.half_height);

        Point::from(vres)
    }
}
