use na::Real;

use math::{Isometry, Point, Vector};
use query::time_of_impact_internal;
use shape::{Ball, Plane, Shape};

/// Computes the smallest time of impact of two shapes under translational movement.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn shape_against_shape<N: Real>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &Shape<N>,
) -> Option<N> {
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        let p1 = Point::from_coordinates(m1.translation.vector);
        let p2 = Point::from_coordinates(m2.translation.vector);

        time_of_impact_internal::ball_against_ball(&p1, vel1, b1, &p2, vel2, b2)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        time_of_impact_internal::plane_against_support_map(m1, vel1, p1, m2, vel2, s2)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        time_of_impact_internal::support_map_against_plane(m1, vel1, s1, m2, vel2, p2)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        time_of_impact_internal::support_map_against_support_map(m1, vel1, s1, m2, vel2, s2)
    } else if let Some(c1) = g1.as_composite_shape() {
        time_of_impact_internal::composite_shape_against_shape(m1, vel1, c1, m2, vel2, g2)
    } else if let Some(c2) = g2.as_composite_shape() {
        time_of_impact_internal::shape_against_composite_shape(m1, vel1, g1, m2, vel2, c2)
    } else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
