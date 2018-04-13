use alga::linear::Translation;
use math::{Isometry, Point};
use shape::{Ball, Plane, Segment, Shape};
use query::closest_points_internal;
use query::closest_points_internal::ClosestPoints;

/// Computes the pair of closest points between two shapes.
///
/// Returns `None` if the objects are separated by a distance greater than `max_dist`.
pub fn shape_against_shape<P, M>(
    m1: &M,
    g1: &Shape<P, M>,
    m2: &M,
    g2: &Shape<P, M>,
    max_dist: P::Real,
) -> ClosestPoints<P>
where
    P: Point,
    M: Isometry<P>,
{
    if let (Some(b1), Some(b2)) = (
        g1.as_shape::<Ball<P::Real>>(),
        g2.as_shape::<Ball<P::Real>>(),
    ) {
        let p1 = P::from_coordinates(m1.translation().to_vector());
        let p2 = P::from_coordinates(m2.translation().to_vector());

        closest_points_internal::ball_against_ball(&p1, b1, &p2, b2, max_dist)
    } else if let (Some(s1), Some(s2)) = (g1.as_shape::<Segment<P>>(), g2.as_shape::<Segment<P>>())
    {
        closest_points_internal::segment_against_segment(m1, s1, m2, s2, max_dist)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<P::Vector>>(), g2.as_support_map()) {
        closest_points_internal::plane_against_support_map(m1, p1, m2, s2, max_dist)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<P::Vector>>()) {
        closest_points_internal::support_map_against_plane(m1, s1, m2, p2, max_dist)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        closest_points_internal::support_map_against_support_map(m1, s1, m2, s2, max_dist)
    } else if let Some(c1) = g1.as_composite_shape() {
        closest_points_internal::composite_shape_against_shape(m1, c1, m2, g2, max_dist)
    } else if let Some(c2) = g2.as_composite_shape() {
        closest_points_internal::shape_against_composite_shape(m1, g1, m2, c2, max_dist)
    } else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
