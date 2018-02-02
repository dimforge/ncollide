//!
//! Support mapping based Cylinder shape.
//!

use num::{Signed, Zero};

use alga::general::Real;
use alga::linear::NormedSpace;
use na;
use shape::SupportMap;
use math::{Isometry, Point};

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Cylinder<N> {
    half_height: N,
    radius: N,
}

impl<N: Real> Cylinder<N> {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `y` axis.
    /// * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: N, radius: N) -> Cylinder<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cylinder {
            half_height: half_height,
            radius: radius,
        }
    }

    /// The cylinder half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height
    }

    /// The radius of the cylinder along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for Cylinder<P::Real> {
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        let local_dir = m.inverse_rotate_vector(dir);

        let mut vres = local_dir;
        let negative = local_dir[1].is_negative();

        vres[1] = na::zero();

        if vres.normalize_mut().is_zero() {
            vres = na::zero()
        } else {
            vres = vres * self.radius();
        }

        if negative {
            vres[1] = -self.half_height()
        } else {
            vres[1] = self.half_height()
        }

        m.transform_point(&P::from_coordinates(vres))
    }
}
