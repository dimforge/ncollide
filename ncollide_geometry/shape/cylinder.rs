//!
//! Support mapping based Cylinder shape.
//!

use num::Signed;
use na::{Rotate, Transform, Norm};
use na;
use shape::SupportMap;
use math::{Scalar, Point, Vector};

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Cylinder<N> {
    half_height: N,
    radius:      N,
}

impl<N> Copy for Cylinder<N> where N: Copy {}

impl<N> Cylinder<N>
    where N: Scalar {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `y` axis.
    /// * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: N, radius: N) -> Cylinder<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cylinder {
            half_height: half_height,
            radius:      radius
        }
    }

    /// The cylinder half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the cylinder along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}


impl<P, M> SupportMap<P, M> for Cylinder<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inverse_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir[1].is_negative();

        vres[1]  = na::zero();

        if na::is_zero(&vres.normalize_mut()) {
            vres = na::zero()
        }
        else {
            vres = vres * self.radius();
        }

        if negative {
            vres[1] = -self.half_height()
        }
        else {
            vres[1] = self.half_height()
        }

        m.transform(&(na::origin::<P>() + vres))
    }
}
