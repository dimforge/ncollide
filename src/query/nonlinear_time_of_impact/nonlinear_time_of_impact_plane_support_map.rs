use na::RealField;

use crate::math::{Isometry, Vector};
use crate::query::{Ray, RayCast};
use crate::shape::Plane;
use crate::shape::SupportMap;

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn time_of_impact_plane_support_map<N, G: ?Sized>(
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
    let vel = *vel_other - *vel_plane;
    let plane_normal = mplane * plane.normal();
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_with_ray(mplane, &Ray::new(closest_point, vel), true)
}

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn time_of_impact_support_map_plane<N, G: ?Sized>(
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
    time_of_impact_plane_support_map(mplane, vel_plane, plane, mother, vel_other, other)
}
