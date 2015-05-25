use na::{Translate, Rotate};
use na;
use entities::support_map::SupportMap;
use entities::shape::Plane;
use math::{Scalar, Point, Vect};

/// Distance between a plane and a support-mapped shape.
pub fn plane_against_support_map<P, M, G: ?Sized>(mplane: &M, plane: &Plane<P::Vect>,
                                                  mother: &M, other: &G)
                                                  -> <P::Vect as Vect>::Scalar
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
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
pub fn support_map_against_plane<P, M, G: ?Sized>(mother: &M, other: &G,
                                                  mplane: &M, plane: &Plane<P::Vect>)
                                                  -> <P::Vect as Vect>::Scalar
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
    plane_against_support_map(mplane, plane, mother, other)
}
