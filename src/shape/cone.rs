//! Support mapping based Cone shape.

use crate::math::{Point, Vector};
use crate::shape::SupportMap;
use na::{self, RealField};

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Cone<N> {
    /// The half-height of the cone.
    pub half_height: N,
    /// The base radius of the cone.
    pub radius: N,
}

impl<N: RealField> Cone<N> {
    /// Creates a new cone.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cone along the `y` axis.
    /// * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: N, radius: N) -> Cone<N> {
        Cone {
            half_height,
            radius,
        }
    }

    /// The cone half length along the `y` axis.
    #[inline]
    #[deprecated(note = "use the `self.half_height` public field directly.")]
    pub fn half_height(&self) -> N {
        self.half_height
    }

    /// The radius of the cone along all but the `y` axis.
    #[inline]
    #[deprecated(note = "use the `self.radius` public field directly.")]
    pub fn radius(&self) -> N {
        self.radius
    }
}

impl<N: RealField> SupportMap<N> for Cone<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        let mut vres = *dir;

        vres[1] = na::zero();

        if vres.normalize_mut().is_zero() {
            vres = na::zero();
            vres[1] = dir[1].copysign(self.half_height);
        } else {
            vres = vres * self.radius;
            vres[1] = -self.half_height;

            if dir.dot(&vres) < dir[1] * self.half_height {
                vres = na::zero();
                vres[1] = self.half_height
            }
        }

        Point::from(vres)
    }
}
