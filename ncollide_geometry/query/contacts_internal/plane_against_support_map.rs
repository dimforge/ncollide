use na::{Translate, Rotate};
use na;
use query::Contact;
use shape::{SupportMap, Plane};
use math::{Point, Vector};

/// Computes one contact between a plane and a support-mapped shape (Cuboid, ConvexHull, etc.)
pub fn plane_against_support_map<P, M, G: ?Sized>(mplane: &M, plane: &Plane<P::Vect>,
                                                  mother: &M, other: &G,
                                                  prediction: <P::Vect as Vector>::Scalar)
                                                  -> Option<Contact<P>>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
    let plane_normal = mplane.rotate(plane.normal());
    let plane_center = mplane.translate(&na::origin());
    let deepest      = other.support_point(mother, &-plane_normal);

    let distance = na::dot(&plane_normal, &(plane_center - deepest));

    if distance > -prediction {
        let c1 = deepest + plane_normal * distance;

        Some(Contact::new(c1, deepest, plane_normal, distance))
    }
    else {
        None
    }
}

/// Computes one contact between a support-mapped shape (Cuboid, ConvexHull, etc.) and a plane.
pub fn support_map_against_plane<P, M, G: ?Sized>(mother: &M, other: &G,
                                                  mplane: &M, plane: &Plane<P::Vect>,
                                                  prediction: <P::Vect as Vector>::Scalar)
                                                  -> Option<Contact<P>>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {
    plane_against_support_map(mplane, plane, mother, other, prediction).map(|mut c| { c.flip(); c })
}


/// Computes a polyhedral approximation of the contact area between a plane and a support-mapped shape.
pub fn plane_against_support_map_manifold<P, M, G: ?Sized>(mplane: &M, plane: &Plane<P::Vect>,
                                                           mother: &M, other: &G,
                                                           prediction:   <P::Vect as Vector>::Scalar,
                                                           eps:          <P::Vect as Vector>::Scalar,
                                                           approx_count: usize,
                                                           out_contacts: &mut Vec<Contact<P>>)
                                                           -> usize
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {

    let plane_normal = mplane.rotate(plane.normal());
    let plane_center = mplane.translate(&na::origin());
    let deepest      = other.support_point(mother, &-plane_normal);

    let distance = na::dot(&plane_normal, &(plane_center - deepest));

    if distance > -prediction {
        let mut supp_pts = Vec::new(); // FIXME: avoid systematic allocation.
        let npts         = other.support_point_set(mother, &-plane_normal, eps, approx_count, &mut supp_pts);

        for p in &supp_pts[..] {
            let distance   = na::dot(&plane_normal, &(plane_center - *p));
            let p_on_plane = *p + plane_normal * distance;
            out_contacts.push(Contact::new(p_on_plane, *p, plane_normal, distance))
        }

        npts
    }
    else {
        0
    }
}

/// Computes a polyhedral approximation of the contact area between a support-mapped shape and a plane.
pub fn support_map_against_plane_manifold<P, M, G: ?Sized>(mother: &M, other: &G,
                                                           mplane: &M, plane: &Plane<P::Vect>,
                                                           prediction:   <P::Vect as Vector>::Scalar,
                                                           eps:          <P::Vect as Vector>::Scalar,
                                                           approx_count: usize,
                                                           out_contacts: &mut Vec<Contact<P>>)
                                                           -> usize
    where P: Point,
          M: Translate<P> + Rotate<P::Vect>,
          G: SupportMap<P, M> {

    let start_idx = out_contacts.len();
    let nctcts = plane_against_support_map_manifold(mplane, plane, mother, other, prediction,
                                                    eps, approx_count, out_contacts);

    for c in &mut out_contacts[start_idx ..] {
        c.flip()
    }

    nctcts
}
