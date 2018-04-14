use alga::linear::Translation;
use na;

use query::Proximity;
use shape::SupportMap;
use shape::Plane;
use math::{Isometry, Point};

/// Proximity between a plane and a support-mapped shape (Cuboid, ConvexHull, etc.)
pub fn plane_against_support_map<P, M, G: ?Sized>(
    mplane: &Isometry<N>,
    plane: &Plane<N>,
    mother: &Isometry<N>,
    other: &G,
    margin: N,
) -> Proximity
where
    N: Real,
    M: Isometry<P>,
    G: SupportMap<N>,
{
    assert!(
        margin >= na::zero(),
        "The proximity margin must be positive or null."
    );

    let plane_normal = mplane.rotate_vector(plane.normal());
    let plane_center = Point::from_coordinates(mplane.translation().to_vector());
    let deepest = other.support_point(mother, &-plane_normal);

    let distance = na::dot(&plane_normal, &(plane_center - deepest));

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
pub fn support_map_against_plane<P, M, G: ?Sized>(
    mother: &Isometry<N>,
    other: &G,
    mplane: &Isometry<N>,
    plane: &Plane<N>,
    margin: N,
) -> Proximity
where
    N: Real,
    M: Isometry<P>,
    G: SupportMap<N>,
{
    plane_against_support_map(mplane, plane, mother, other, margin)
}
