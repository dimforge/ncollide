use na::{Rotate, Transform};
use entities::support_map::SupportMap;
use entities::shape::Plane;
use ray::{Ray, RayCast};
use math::{Scalar, Point, Vect};

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn plane_against_support_map<N, P, V, M, G: ?Sized>(mplane: &M, vel_plane: &V, plane: &Plane<V>,
                                                        mother: &M, vel_other: &V, other: &G)
                                                        -> Option<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Rotate<V> + Transform<P>,
          G: SupportMap<P, V, M> {
    let vel           = *vel_other - *vel_plane;
    let plane_normal  = mplane.rotate(plane.normal());
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_with_transform_and_ray(mplane, &Ray::new(closest_point, vel.clone()), true)
}

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn support_map_against_plane<N, P, V, M, G: ?Sized>(mother: &M, vel_other: &V, other: &G,
                                                        mplane: &M, vel_plane: &V, plane: &Plane<V>)
                                                        -> Option<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Rotate<V> + Transform<P>,
          G: SupportMap<P, V, M> {
    plane_against_support_map(mplane, vel_plane, plane, mother, vel_other, other)
}
