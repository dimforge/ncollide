use shape::SupportMap;
use shape::Plane;
use query::{Ray, RayCast};
use math::{Point, Isometry};

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn plane_against_support_map<P, M, G: ?Sized>(mplane: &M, vel_plane: &P::Vector, plane: &Plane<P::Vector>,
                                                  mother: &M, vel_other: &P::Vector, other: &G)
                                                  -> Option<P::Real>
    where P: Point,
          M: Isometry<P>,
          G: SupportMap<P, M> {
    let vel           = *vel_other - *vel_plane;
    let plane_normal  = mplane.rotate_vector(plane.normal());
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_with_ray(mplane, &Ray::new(closest_point, vel), true)
}

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn support_map_against_plane<P, M, G: ?Sized>(mother: &M, vel_other: &P::Vector, other: &G,
                                                  mplane: &M, vel_plane: &P::Vector, plane: &Plane<P::Vector>)
                                                  -> Option<P::Real>
    where P: Point,
          M: Isometry<P>,
          G: SupportMap<P, M> {
    plane_against_support_map(mplane, vel_plane, plane, mother, vel_other, other)
}
