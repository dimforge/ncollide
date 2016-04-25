use na::Translate;
use na;
use math::{Point, Vector, Isometry};
use shape::{Shape, Ball, Plane};
use query::time_of_impact_internal;

/// Computes the smallest time of impact of two shapes under translational movement.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn any_against_any<P, M>(m1: &M, vel1: &P::Vect, g1: &Shape<P, M>,
                             m2: &M, vel2: &P::Vect, g2: &Shape<P, M>)
                             -> Option<<P::Vect as Vector>::Scalar>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P> {
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<<P::Vect as Vector>::Scalar>>(),
                                   g2.as_shape::<Ball<<P::Vect as Vector>::Scalar>>()) {
        let p1 = m1.translate(&na::origin());
        let p2 = m2.translate(&na::origin());

        time_of_impact_internal::ball_against_ball(&p1, vel1, b1, &p2, vel2, b2)
    }
    else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<P::Vect>>(), g2.as_support_map()) {
        time_of_impact_internal::plane_against_support_map(m1, vel1, p1, m2, vel2, s2)
    }
    else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<P::Vect>>()) {
        time_of_impact_internal::support_map_against_plane(m1, vel1, s1, m2, vel2, p2)
    }
    else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        time_of_impact_internal::support_map_against_support_map(m1, vel1, s1, m2, vel2, s2)
    }
    else if let Some(c1) = g1.as_composite_shape() {
        time_of_impact_internal::composite_shape_against_any(m1, vel1, c1, m2, vel2, g2)
    }
    else if let Some(c2) = g2.as_composite_shape() {
        time_of_impact_internal::any_against_composite_shape(m1, vel1, g1, m2, vel2, c2)
    }
    else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
