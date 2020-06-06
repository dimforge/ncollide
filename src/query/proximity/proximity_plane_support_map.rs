use na::{self, RealField};

use crate::math::{Isometry, Point};
use crate::query::Proximity;
use crate::shape::Plane;
use crate::shape::SupportMap;

/// Proximity between a plane and a support-mapped shape (Cuboid, ConvexHull, etc.)
pub fn proximity_plane_support_map<N: RealField, G: ?Sized + SupportMap<N>>(
    mplane: &Isometry<N>,
    plane: &Plane<N>,
    mother: &Isometry<N>,
    other: &G,
    margin: N,
) -> Proximity {
    assert!(
        margin >= na::zero(),
        "The proximity margin must be positive or null."
    );

    let plane_normal = mplane * plane.normal;
    let plane_center = Point::from(mplane.translation.vector);
    let deepest = other.support_point_toward(mother, &-plane_normal);

    let distance = plane_normal.dot(&(plane_center - deepest));

    if distance >= -margin {
        if distance >= na::zero() {
            Proximity::Intersecting
        } else {
            Proximity::WithinMargin
        }
    } else {
        Proximity::Disjoint
    }
}

/// Proximity between a support-mapped shape (Cuboid, ConvexHull, etc.) and a plane.
pub fn proximity_support_map_plane<N: RealField, G: ?Sized + SupportMap<N>>(
    mother: &Isometry<N>,
    other: &G,
    mplane: &Isometry<N>,
    plane: &Plane<N>,
    margin: N,
) -> Proximity {
    proximity_plane_support_map(mplane, plane, mother, other, margin)
}
