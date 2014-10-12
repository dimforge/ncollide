use na::{FloatPnt, FloatPntExt, FloatVec, Cast, Norm};
use na;
use utils;

/// Computes the bounding sphere of a set of point, given its center.
// FIXME: return a bounding sphere?
#[inline]
pub fn point_cloud_bounding_sphere_with_center<N: Float, P: FloatPnt<N, V>, V: Norm<N>>(pts: &[P], center: P) -> (P, N) {
    let mut sqradius = na::zero();

    for pt in pts.iter() {
        let sqdist = na::sqdist(pt, &center);

        if sqdist > sqradius {
            sqradius = sqdist
        }
    }

    (center, sqradius.sqrt())
}


/// Computes a bounding sphere of the specified set of point.
// FIXME: return a bounding sphere?
#[inline]
pub fn point_cloud_bounding_sphere<N: Float + Cast<f64>, P: FloatPntExt<N, V>, V: FloatVec<N>>(pts: &[P]) -> (P, N) {
    point_cloud_bounding_sphere_with_center(pts, utils::center(pts))
}
