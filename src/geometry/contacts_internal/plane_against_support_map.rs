use na::{Translate, Rotate};
use na;
use geometry::Contact;
use support_map::SupportMap;
use shape::Plane;
use math::{Scalar, Point, Vect};

/// Contact between a plane and a support-mapped shape (Cuboid, Convex, etc.)
pub fn plane_against_support_map<N, P, V, M, G>(mplane: &M, plane: &Plane<V>,
                                                mother: &M, other: &G,
                                                prediction: N)
                                                -> Option<Contact<N, P, V>>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Rotate<V>,
          G: SupportMap<P, V, M> {
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
pub fn support_map_against_plane<N, P, V, M, G>(mother: &M, other: &G,
                                                mplane: &M, plane: &Plane<V>,
                                                prediction: N)
                                                -> Option<Contact<N, P, V>>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Rotate<V>,
          G: SupportMap<P, V, M> {
    plane_against_support_map(mplane, plane, mother, other, prediction).map(|mut c| { c.flip(); c })
}
