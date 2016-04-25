use na::{Translate, Rotate};
use na;
use shape::SupportMap;
use shape::Plane;
use math::{Point, Vector};

/// Distance between a plane and a support-mapped shape.
pub fn plane_against_support_map<P, M, G: ?Sized>(mplane: &M, plane: &Plane<P::Vect>,
                                                  mother: &M, other: &G)
                                                  -> <P::Vect as Vector>::Scalar
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
    let plane_normal = mplane.rotate(plane.normal());
    let plane_center = mplane.translate(&na::origin());
    let deepest      = other.support_point(mother, &-plane_normal);

    let distance = na::dot(&plane_normal, &(plane_center - deepest));

    if distance < na::zero() {
        -distance
    }
    else {
        na::zero()
    }
}

/// Distance between a support-mapped shape and a plane.
pub fn support_map_against_plane<P, M, G: ?Sized>(mother: &M, other: &G,
                                                  mplane: &M, plane: &Plane<P::Vect>)
                                                  -> <P::Vect as Vector>::Scalar
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
    plane_against_support_map(mplane, plane, mother, other)
}
