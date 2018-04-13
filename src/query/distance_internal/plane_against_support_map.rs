use alga::linear::Translation;
use na;
use shape::SupportMap;
use shape::Plane;
use math::{Isometry, Point};

/// Distance between a plane and a support-mapped shape.
pub fn plane_against_support_map<P, M, G: ?Sized>(
    mplane: &M,
    plane: &Plane<P::Vector>,
    mother: &M,
    other: &G,
) -> P::Real
where
    P: Point,
    M: Isometry<P>,
    G: SupportMap<P, M>,
{
    let plane_normal = mplane.rotate_vector(plane.normal());
    let plane_center = P::from_coordinates(mplane.translation().to_vector());
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
    mother: &M,
    other: &G,
    mplane: &M,
    plane: &Plane<P::Vector>,
) -> P::Real
where
    P: Point,
    M: Isometry<P>,
    G: SupportMap<P, M>,
{
    plane_against_support_map(mplane, plane, mother, other)
}
