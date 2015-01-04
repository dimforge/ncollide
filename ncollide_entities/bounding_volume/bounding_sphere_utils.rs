use na::Norm;
use na;
use utils;
use math::{Scalar, Point, Vect};

/// Computes the bounding sphere of a set of point, given its center.
// FIXME: return a bounding sphere?
#[inline]
pub fn point_cloud_bounding_sphere_with_center<N, P, V>(pts: &[P], center: P) -> (P, N)
    where N: Scalar,
          P: Point<N, V>,
          V: Norm<N> {
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
pub fn point_cloud_bounding_sphere<N, P, V>(pts: &[P]) -> (P, N)
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    point_cloud_bounding_sphere_with_center(pts, utils::center(pts))
}
