use na::{Rotate, Transform};
use support_map::SupportMap;
use shape::Plane;
use ray::{Ray, RayCast};
use math::{Scalar, Point, Vect};

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn plane_against_support_map<N, P, V, M, G>(mplane: &M, dir_plane: &V, plane: &Plane<V>,
                                                mother: &M, dir_other: &V, other: &G)
                                                -> Option<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Rotate<V> + Transform<P>,
          G: SupportMap<P, V, M> {
    let dir           = *dir_other - *dir_plane;
    let plane_normal  = mplane.rotate(plane.normal());
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_with_transform_and_ray(mplane, &Ray::new(closest_point, dir.clone()), true)
}

/// Time Of Impact of a plane with a support-mapped shape under translational movement.
pub fn support_map_against_plane<N, P, V, M, G>(mother: &M, dir_other: &V, other: &G,
                                                mplane: &M, dir_plane: &V, plane: &Plane<V>)
                                                -> Option<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Rotate<V> + Transform<P>,
          G: SupportMap<P, V, M> {
    plane_against_support_map(mplane, dir_plane, plane, mother, dir_other, other)
}
