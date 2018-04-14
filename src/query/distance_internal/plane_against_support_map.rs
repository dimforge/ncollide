use alga::linear::Translation;
use na;
use shape::SupportMap;
use shape::Plane;
use math::{Isometry, Point};

/// Distance between a plane and a support-mapped shape.
pub fn plane_against_support_map<P, M, G: ?Sized>(
    mplane: &Isometry<N>,
    plane: &Plane<N>,
    mother: &Isometry<N>,
    other: &G,
) -> N
where
    N: Real,
    M: Isometry<P>,
    G: SupportMap<N>,
{
    let plane_normal = mplane.rotate_vector(plane.normal());
    let plane_center = Point::from_coordinates(mplane.translation().to_vector());
    let deepest = other.support_point(mother, &-plane_normal);

    let distance = na::dot(&plane_normal, &(plane_center - deepest));

    if distance < na::zero() {
        -distance
    } else {
        na::zero()
    }
}

/// Distance between a support-mapped shape and a plane.
pub fn support_map_against_plane<P, M, G: ?Sized>(
    mother: &Isometry<N>,
    other: &G,
    mplane: &Isometry<N>,
    plane: &Plane<N>,
) -> N
where
    N: Real,
    M: Isometry<P>,
    G: SupportMap<N>,
{
    plane_against_support_map(mplane, plane, mother, other)
}
