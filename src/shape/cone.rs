//! Support mapping based Cone shape.

use na::{self, Real};
use utils::IsometryOps;
use shape::SupportMap;
use math::{Point, Vector, Isometry};

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone)]
pub struct Cone<N> {
    half_height: N,
    radius: N,
}

impl<N: Real> Cone<N> {
    /// Creates a new cone.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cone along the `y` axis.
    /// * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: N, radius: N) -> Cone<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cone {
            half_height: half_height,
            radius: radius,
        }
    }

    /// The cone half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height
    }

    /// The radius of the cone along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius
    }
}

impl<N: Real> SupportMap<N> for Cone<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        let mut vres = *dir;

        vres[1] = na::zero();

        if vres.normalize_mut().is_zero() {
            vres = na::zero();

            if dir[1].is_negative() {
                vres[1] = -self.half_height()
            } else {
                vres[1] = self.half_height()
            }
        } else {
            vres = vres * self.radius();
            vres[1] = -self.half_height();

            if na::dot(dir, &vres) < dir[1] * self.half_height() {
                vres = na::zero();
                vres[1] = self.half_height()
            }
        }

        Point::from_coordinates(vres)
    }
}
