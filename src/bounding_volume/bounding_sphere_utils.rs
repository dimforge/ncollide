use nalgebra::na;
use math::{Vect, Scalar};

/// Computes the center of a set of point.
#[inline]
pub fn center(pts: &[Vect]) -> Vect {
    let mut res       = na::zero::<Vect>();
    let denom: Scalar = na::cast(1.0 / (pts.len() as f64));

    for pt in pts.iter() {
        res = res + *pt * denom;
    }

    res
}

/// Computes the bounding sphere of a set of point, given its center.
// FIXME: return a bounding sphere?
#[inline]
pub fn bounding_sphere_with_center(pts: &[Vect], center: Vect) -> (Vect, Scalar) {
    let mut sqradius = na::zero();

    for pt in pts.iter() {
        let sqnorm = na::sqnorm(&(*pt - center));

        if sqnorm > sqradius {
            sqradius = sqnorm
        }
    }

    (center, sqradius.sqrt())
}


/// Computes a bounding sphere of the specified set of point.
// FIXME: return a bounding sphere?
#[inline]
pub fn bounding_sphere(pts: &[Vect]) -> (Vect, Scalar) {
    bounding_sphere_with_center(pts, center(pts))
}
