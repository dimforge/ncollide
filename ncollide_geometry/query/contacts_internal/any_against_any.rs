use na;
use na::{Translate, Translation};
use math::{Point, Vector, Isometry};
use shape::{Shape, Ball, Plane};
use query::contacts_internal;
use query::contacts_internal::Contact;

/// Computes one contact point between two shapes.
///
/// Returns `None` if the objects are separated by a distance greater than `prediction`.
pub fn any_against_any<P, M>(m1: &M, g1: &Shape<P, M>,
                             m2: &M, g2: &Shape<P, M>,
                             prediction: <P::Vect as Vector>::Scalar)
                             -> Option<Contact<P>>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P> + Translation<P::Vect> {
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<<P::Vect as Vector>::Scalar>>(),
                                   g2.as_shape::<Ball<<P::Vect as Vector>::Scalar>>()) {
        let p1 = m1.translate(&na::origin());
        let p2 = m2.translate(&na::origin());

        contacts_internal::ball_against_ball(&p1, b1, &p2, b2, prediction)
    }
    else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<P::Vect>>(), g2.as_support_map()) {
        contacts_internal::plane_against_support_map(m1, p1, m2, s2, prediction)
    }
    else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<P::Vect>>()) {
        contacts_internal::support_map_against_plane(m1, s1, m2, p2, prediction)
    }
    else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        contacts_internal::support_map_against_support_map(m1, s1, m2, s2, prediction)
    }
    else if let Some(c1) = g1.as_composite_shape() {
        contacts_internal::composite_shape_against_any(m1, c1, m2, g2, prediction)
    }
    else if let Some(c2) = g2.as_composite_shape() {
        contacts_internal::any_against_composite_shape(m1, g1, m2, c2, prediction)
    }
    else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
