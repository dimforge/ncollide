//! Support mapping based Capsule shape.

use num::Signed;
use na;
use na::{Rotate, Transform};
use shape::SupportMap;
use math::{Scalar, Point, Vector};

/// SupportMap description of a capsule shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Capsule<N> {
    half_height: N,
    radius:      N,
}

impl<N> Copy for Capsule<N> where N: Copy {}

impl<N> Capsule<N>
    where N: Scalar {
    /// Creates a new capsule.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the capsule along the `y` axis.
    /// * `radius` - radius of the rounded part of the capsule.
    pub fn new(half_height: N, radius: N) -> Capsule<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Capsule {
            half_height: half_height,
            radius:      radius,
        }
    }

    /// The capsule half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the capsule's rounded part.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}

impl<P, M> SupportMap<P, M> for Capsule<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inverse_rotate(dir);

        let mut pres = na::origin::<P>();

        if local_dir[1].is_negative() {
            pres[1] = -self.half_height()
        }
        else {
            pres[1] = self.half_height()
        }

        m.transform(&(pres + na::normalize(&local_dir) * self.radius()))
    }
}
