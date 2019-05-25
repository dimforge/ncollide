use na::RealField;

use crate::math::{Isometry, Vector};
use crate::query::{Ray, RayCast};
use crate::shape::Plane;
use crate::shape::SupportMap;

/// Time of impact of a plane with a support-mapped shape under translational movement.
pub fn plane_against_support_map<N, G: ?Sized>(
    mplane: &Isometry<N>,
    vel_plane: &Vector<N>,
    plane: &Plane<N>,
    mother: &Isometry<N>,
    vel_other: &Vector<N>,
    other: &G,
) -> Option<N>
where
    N: RealField,
    G: SupportMap<N>,
{
    plane_against_support_map_with_normal(mplane, vel_plane, plane, mother, vel_other, other).map(|x| x.0)
}

/// Time of impact and contact normal of a plane with a support-mapped shape
/// under translational movement.
pub fn plane_against_support_map_with_normal<N, G: ?Sized>(
    mplane: &Isometry<N>,
    vel_plane: &Vector<N>,
    plane: &Plane<N>,
    mother: &Isometry<N>,
    vel_other: &Vector<N>,
    other: &G,
) -> Option<(N, Vector<N>)>
where
    N: RealField,
    G: SupportMap<N>,
{
    let vel = *vel_other - *vel_plane;
    let plane_normal = mplane * plane.normal();
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_and_normal_with_ray(mplane, &Ray::new(closest_point, vel), true).map(|x| (x.toi, x.normal))
}

/// Time of impact of a plane with a support-mapped shape under translational movement.
pub fn support_map_against_plane<N, G: ?Sized>(
    mother: &Isometry<N>,
    vel_other: &Vector<N>,
    other: &G,
    mplane: &Isometry<N>,
    vel_plane: &Vector<N>,
    plane: &Plane<N>,
) -> Option<N>
where
    N: RealField,
    G: SupportMap<N>,
{
    plane_against_support_map(mplane, vel_plane, plane, mother, vel_other, other)
}

/// Time of impact and contact normal of a plane with a support-mapped shape
/// under translational movement.
pub fn support_map_against_plane_with_normal<N, G: ?Sized>(
    mother: &Isometry<N>,
    vel_other: &Vector<N>,
    other: &G,
    mplane: &Isometry<N>,
    vel_plane: &Vector<N>,
    plane: &Plane<N>,
) -> Option<(N, Vector<N>)>
where
    N: RealField,
    G: SupportMap<N>,
{
    plane_against_support_map_with_normal(mplane, vel_plane, plane, mother, vel_other, other).map(|x| (x.0, -x.1))
}
