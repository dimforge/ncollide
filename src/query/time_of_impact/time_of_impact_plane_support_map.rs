use na::RealField;

use crate::math::{Isometry, Vector};
use crate::query::{Ray, RayCast, TOIStatus, TOI};
use crate::shape::{Plane, SupportMap};

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn time_of_impact_plane_support_map<N, G: ?Sized>(
    mplane: &Isometry<N>,
    vel_plane: &Vector<N>,
    plane: &Plane<N>,
    mother: &Isometry<N>,
    vel_other: &Vector<N>,
    other: &G,
    max_toi: N,
    target_distance: N,
) -> Option<TOI<N>>
where
    N: RealField + Copy,
    G: SupportMap<N>,
{
    let vel = *vel_other - *vel_plane;
    let plane_normal = mplane * plane.normal;
    // FIXME: add method to get only the local support point.
    // This would avoid the `inverse_transform_point` later.
    let support_point = other.support_point(mother, &-plane_normal);
    let closest_point = support_point - *plane_normal * target_distance;
    let ray = Ray::new(closest_point, vel);

    if let Some(toi) = plane.toi_with_ray(mplane, &ray, max_toi, true) {
        if toi > max_toi {
            return None;
        }

        let status;
        let witness1 = mother.inverse_transform_point(&support_point);
        let mut witness2 = mplane.inverse_transform_point(&ray.point_at(toi));

        if (support_point.coords - mplane.translation.vector).dot(&plane_normal) < N::zero() {
            status = TOIStatus::Penetrating
        } else {
            // Project the witness point to the plane.
            // Note that witness2 is already in the plane's local-space.
            witness2 = witness2 - *plane.normal * witness2.coords.dot(&plane.normal);
            status = TOIStatus::Converged
        }

        Some(TOI {
            toi,
            normal1: plane.normal,
            normal2: mother.inverse_transform_unit_vector(&-plane_normal),
            witness1,
            witness2,
            status,
        })
    } else {
        None
    }
}

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn time_of_impact_support_map_plane<N, G: ?Sized>(
    mother: &Isometry<N>,
    vel_other: &Vector<N>,
    other: &G,
    mplane: &Isometry<N>,
    vel_plane: &Vector<N>,
    plane: &Plane<N>,
    max_toi: N,
    target_distance: N,
) -> Option<TOI<N>>
where
    N: RealField + Copy,
    G: SupportMap<N>,
{
    time_of_impact_plane_support_map(
        mplane,
        vel_plane,
        plane,
        mother,
        vel_other,
        other,
        max_toi,
        target_distance,
    )
    .map(|toi| toi.swapped())
}
