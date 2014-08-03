use nalgebra::na::{FloatVec, Cast};
use nalgebra::na;
use utils;

/// Computes the bounding sphere of a set of point, given its center.
// FIXME: return a bounding sphere?
#[inline]
pub fn point_cloud_bounding_sphere_with_center<N: Float, V: FloatVec<N>>(pts: &[V], center: V) -> (V, N) {
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
pub fn point_cloud_bounding_sphere<N: Float + Cast<f64>, V: FloatVec<N>>(pts: &[V]) -> (V, N) {
    point_cloud_bounding_sphere_with_center(pts, utils::center(pts))
}
