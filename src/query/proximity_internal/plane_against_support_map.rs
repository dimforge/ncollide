use na::{self, Real};

use query::Proximity;
use shape::SupportMap;
use shape::Plane;
use math::{Isometry, Point};

/// Proximity between a plane and a support-mapped shape (Cuboid, ConvexHull, etc.)
pub fn plane_against_support_map<N: Real, G: ?Sized + SupportMap<N>>(
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

    let plane_normal = mplane * plane.normal();
    let plane_center = Point::from_coordinates(mplane.translation.vector);
    let deepest = other.support_point_toward(mother, &-plane_normal);

    let distance = na::dot(&*plane_normal, &(plane_center - deepest));

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
pub fn support_map_against_plane<N: Real, G: ?Sized + SupportMap<N>>(
    mother: &Isometry<N>,
    other: &G,
    mplane: &Isometry<N>,
    plane: &Plane<N>,
    margin: N,
) -> Proximity {
    plane_against_support_map(mplane, plane, mother, other, margin)
}
