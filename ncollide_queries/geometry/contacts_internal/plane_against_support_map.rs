use na::{Translate, Rotate};
use na;
use geometry::Contact;
use entities::support_map::SupportMap;
use entities::shape::Plane;
use math::{Scalar, Point, Vect};

/// Contact between a plane and a support-mapped shape (Cuboid, Convex, etc.)
pub fn plane_against_support_map<P, M, G: ?Sized>(mplane: &M, plane: &Plane<P::Vect>,
                                                  mother: &M, other: &G,
                                                  prediction: <P::Vect as Vect>::Scalar)
                                                  -> Option<Contact<P>>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
    let plane_normal = mplane.rotate(plane.normal());
    let plane_center = mplane.translate(&na::orig());
    let deepest      = other.support_point(mother, &-plane_normal);

    let dist = na::dot(&plane_normal, &(plane_center - deepest));

    if dist > -prediction {
        let c1 = deepest + plane_normal * dist;

        Some(Contact::new(c1, deepest, plane_normal, dist))
    }
    else {
        None
    }
}

/// Contact between a support-mapped shape (Cuboid, Convex, etc.) and a plane.
pub fn support_map_against_plane<P, M, G: ?Sized>(mother: &M, other: &G,
                                                  mplane: &M, plane: &Plane<P::Vect>,
                                                  prediction: <P::Vect as Vect>::Scalar)
                                                  -> Option<Contact<P>>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
    plane_against_support_map(mplane, plane, mother, other, prediction).map(|mut c| { c.flip(); c })
}
