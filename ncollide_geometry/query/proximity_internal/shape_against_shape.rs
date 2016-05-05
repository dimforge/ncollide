use na;
use na::{Translate, Translation};
use math::{Point, Vector, Isometry};
use shape::{Shape, Ball, Plane};
use query::Proximity;
use query::proximity_internal;

/// Tests whether two shapes are in intersecting or separated by a distance smaller than `margin`.
pub fn shape_against_shape<P, M>(m1: &M, g1: &Shape<P, M>,
                             m2: &M, g2: &Shape<P, M>,
                             margin: <P::Vect as Vector>::Scalar)
                             -> Proximity
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Isometry<P> + Translation<P::Vect> {
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<<P::Vect as Vector>::Scalar>>(),
                                   g2.as_shape::<Ball<<P::Vect as Vector>::Scalar>>()) {
        let p1 = m1.translate(&na::origin());
        let p2 = m2.translate(&na::origin());

        proximity_internal::ball_against_ball(&p1, b1, &p2, b2, margin)
    }
    else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<P::Vect>>(), g2.as_support_map()) {
        proximity_internal::plane_against_support_map(m1, p1, m2, s2, margin)
    }
    else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<P::Vect>>()) {
        proximity_internal::support_map_against_plane(m1, s1, m2, p2, margin)
    }
    else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        proximity_internal::support_map_against_support_map::<P, _, _, _>(m1, s1, m2, s2, margin)
    }
    else if let Some(c1) = g1.as_composite_shape() {
        proximity_internal::composite_shape_against_shape(m1, c1, m2, g2, margin)
    }
    else if let Some(c2) = g2.as_composite_shape() {
        proximity_internal::shape_against_composite_shape(m1, g1, m2, c2, margin)
    }
    else {
        panic!("No algorithm known to compute proximity between the given pair of shapes.")
    }
}
