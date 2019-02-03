use crate::math::{Isometry, Point};
use na::Real;
use crate::query::distance_internal;
use crate::shape::{Ball, Plane, Shape};

/// Computes the minimum distance separating two shapes.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn shape_against_shape<N: Real>(
    m1: &Isometry<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    g2: &Shape<N>,
) -> N
{
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        let p1 = Point::from(m1.translation.vector);
        let p2 = Point::from(m2.translation.vector);

        distance_internal::ball_against_ball(&p1, b1, &p2, b2)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        distance_internal::plane_against_support_map(m1, p1, m2, s2)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        distance_internal::support_map_against_plane(m1, s1, m2, p2)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        distance_internal::support_map_against_support_map(m1, s1, m2, s2)
    } else if let Some(c1) = g1.as_composite_shape() {
        distance_internal::composite_shape_against_shape(m1, c1, m2, g2)
    } else if let Some(c2) = g2.as_composite_shape() {
        distance_internal::shape_against_composite_shape(m1, g1, m2, c2)
    } else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
