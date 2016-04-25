use num::Float;
use na;
use utils;
use math::{Point, Vector};

/// Computes the bounding sphere of a set of point, given its center.
// FIXME: return a bounding sphere?
#[inline]
pub fn point_cloud_bounding_sphere_with_center<P>(pts: &[P], center: P) -> (P, <P::Vect as Vector>::Scalar)
    where P: Point {
    let mut sqradius = na::zero();

    for pt in pts.iter() {
        let distance_squared = na::distance_squared(pt, &center);

        if distance_squared > sqradius {
            sqradius = distance_squared
        }
    }

    (center, sqradius.sqrt())
}


/// Computes a bounding sphere of the specified set of point.
// FIXME: return a bounding sphere?
#[inline]
pub fn point_cloud_bounding_sphere<P>(pts: &[P]) -> (P, <P::Vect as Vector>::Scalar)
    where P: Point {
    point_cloud_bounding_sphere_with_center(pts, utils::center(pts))
}
