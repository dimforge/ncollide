use shape::SupportMap;
use shape::Plane;
use query::{Ray, RayCast};
use math::{Isometry, Point};

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn plane_against_support_map<N, G: ?Sized>(
    mplane: &Isometry<N>,
    vel_plane: &Vector<N>,
    plane: &Plane<N>,
    mother: &Isometry<N>,
    vel_other: &Vector<N>,
    other: &G,
) -> Option<N>
where
    N: Real,
    M: Isometry<P>,
    G: SupportMap<N>,
{
    let vel = *vel_other - *vel_plane;
    let plane_normal = mplane.rotate_vector(plane.normal());
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_with_ray(mplane, &Ray::new(closest_point, vel), true)
}

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn support_map_against_plane<N, G: ?Sized>(
    mother: &Isometry<N>,
    vel_other: &Vector<N>,
    other: &G,
    mplane: &Isometry<N>,
    vel_plane: &Vector<N>,
    plane: &Plane<N>,
) -> Option<N>
where
    N: Real,
    M: Isometry<P>,
    G: SupportMap<N>,
{
    plane_against_support_map(mplane, vel_plane, plane, mother, vel_other, other)
}
