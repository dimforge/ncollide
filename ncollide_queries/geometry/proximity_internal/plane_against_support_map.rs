use na::{Translate, Rotate};
use na;
use geometry::Proximity;
use entities::support_map::SupportMap;
use entities::shape::Plane;
use math::{Point, Vect};

/// Proximity between a plane and a support-mapped shape (Cuboid, Convex, etc.)
pub fn plane_against_support_map<P, M, G: ?Sized>(mplane: &M, plane: &Plane<P::Vect>,
                                                  mother: &M, other: &G,
                                                  margin: <P::Vect as Vect>::Scalar)
                                                  -> Proximity
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
    let plane_normal = mplane.rotate(plane.normal());
    let plane_center = mplane.translate(&na::orig());
    let deepest      = other.support_point(mother, &-plane_normal);

    let dist = na::dot(&plane_normal, &(plane_center - deepest));

    if dist >= -margin {
        if dist >= na::zero() {
            Proximity::Intersecting
        }
        else {
            Proximity::WithinMargin
        }
    }
    else {
        Proximity::Disjoint
    }
}

/// Proximity between a support-mapped shape (Cuboid, Convex, etc.) and a plane.
pub fn support_map_against_plane<P, M, G: ?Sized>(mother: &M, other: &G,
                                                  mplane: &M, plane: &Plane<P::Vect>,
                                                  margin: <P::Vect as Vect>::Scalar)
                                                  -> Proximity
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
    plane_against_support_map(mplane, plane, mother, other, margin)
}
