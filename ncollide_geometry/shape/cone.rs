//! Support mapping based Cone shape.

use num::Signed;
use na::{Rotate, Transform, Norm};
use na;
use shape::SupportMap;
use math::{Scalar, Point, Vector};

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Cone<N> {
    half_height: N,
    radius:      N,
}

impl<N> Cone<N>
    where N: Scalar {
    /// Creates a new cone.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cone along the `y` axis.
    /// * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: N, radius: N) -> Cone<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cone {
            half_height: half_height,
            radius:      radius
        }
    }

    /// The cone half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the cone along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}


impl<P, M> SupportMap<P, M> for Cone<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inverse_rotate(dir);

        let mut vres = local_dir.clone();

        vres[1] = na::zero();

        if na::is_zero(&vres.normalize_mut()) {
            vres = na::zero();

            if local_dir[1].is_negative() {
                vres[1] = -self.half_height()
            }
            else {
                vres[1] = self.half_height()
            }
        }
        else {
            vres = vres * self.radius();
            vres[1] = -self.half_height();

            if na::dot(&local_dir, &vres) < local_dir[1] * self.half_height() {
                vres = na::zero();
                vres[1] = self.half_height()
            }
        }

        m.transform(&(na::origin::<P>() + vres))
    }
}
