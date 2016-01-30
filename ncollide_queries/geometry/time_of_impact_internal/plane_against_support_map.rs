use na::{Rotate, Transform};
use entities::support_map::SupportMap;
use entities::shape::Plane;
use ray::{Ray, RayCast};
use math::{Point, Vect};

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn plane_against_support_map<P, M, G: ?Sized>(mplane: &M, vel_plane: &P::Vect, plane: &Plane<P::Vect>,
                                                  mother: &M, vel_other: &P::Vect, other: &G)
                                                  -> Option<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Rotate<P::Vect> + Transform<P>,
          G: SupportMap<P, M> {
    let vel           = *vel_other - *vel_plane;
    let plane_normal  = mplane.rotate(plane.normal());
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_with_ray(mplane, &Ray::new(closest_point, vel.clone()), true)
}

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn support_map_against_plane<P, M, G: ?Sized>(mother: &M, vel_other: &P::Vect, other: &G,
                                                  mplane: &M, vel_plane: &P::Vect, plane: &Plane<P::Vect>)
                                                  -> Option<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Rotate<P::Vect> + Transform<P>,
          G: SupportMap<P, M> {
    plane_against_support_map(mplane, vel_plane, plane, mother, vel_other, other)
}
