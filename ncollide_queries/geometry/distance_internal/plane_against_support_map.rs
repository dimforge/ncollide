use na::{Translate, Rotate};
use na;
use entities::support_map::SupportMap;
use entities::shape::Plane;
use math::{Scalar, Point, Vect};

/// Distance between a plane and a support-mapped shape.
pub fn plane_against_support_map<N, P, V, M, Sized? G>(mplane: &M, plane: &Plane<V>,
                                                       mother: &M, other: &G)
                                                       -> N
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Rotate<V>,
          G: SupportMap<P, V, M> {
    let plane_normal = mplane.rotate(plane.normal());
    let plane_center = mplane.translate(&na::orig());
    let deepest      = other.support_point(mother, &-plane_normal);

    let dist = na::dot(&plane_normal, &(plane_center - deepest));

    if dist < na::zero() {
        -dist
    }
    else {
        na::zero()
    }
}

/// Distance between a support-mapped shape and a plane.
pub fn support_map_against_plane<N, P, V, M, Sized? G>(mother: &M, other: &G,
                                                       mplane: &M, plane: &Plane<V>)
                                                       -> N
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Rotate<V>,
          G: SupportMap<P, V, M> {
    plane_against_support_map(mplane, plane, mother, other)
}
