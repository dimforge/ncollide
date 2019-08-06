use crate::math::{Isometry, Point};
use na::{self, RealField};
use crate::shape::Plane;
use crate::shape::SupportMap;

/// Distance between a plane and a support-mapped shape.
pub fn distance_plane_support_map<N: RealField, G: ?Sized + SupportMap<N>>(
    mplane: &Isometry<N>,
    plane: &Plane<N>,
    mother: &Isometry<N>,
    other: &G,
) -> N
{
    let plane_normal = mplane * plane.normal();
    let plane_center = Point::from(mplane.translation.vector);
    let deepest = other.support_point_toward(mother, &-plane_normal);

    let distance = plane_normal.dot(&(plane_center - deepest));

    if distance < na::zero() {
        -distance
    } else {
        na::zero()
    }
}

/// Distance between a support-mapped shape and a plane.
pub fn distance_support_map_plane<N: RealField, G: ?Sized + SupportMap<N>>(
    mother: &Isometry<N>,
    other: &G,
    mplane: &Isometry<N>,
    plane: &Plane<N>,
) -> N
{
    distance_plane_support_map(mplane, plane, mother, other)
}
