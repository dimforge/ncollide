use na::Real;

use math::{Isometry, Point};
use query::closest_points_internal;
use query::closest_points_internal::ClosestPoints;
use shape::{Ball, Plane, Segment, Shape};

/// Computes the pair of closest points between two shapes.
///
/// Returns `None` if the objects are separated by a distance greater than `max_dist`.
pub fn shape_against_shape<N: Real>(
    m1: &Isometry<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    g2: &Shape<N>,
    max_dist: N,
) -> ClosestPoints<N> {
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        let p1 = Point::from_coordinates(m1.translation.vector);
        let p2 = Point::from_coordinates(m2.translation.vector);

        closest_points_internal::ball_against_ball(&p1, b1, &p2, b2, max_dist)
    } else if let (Some(s1), Some(s2)) = (g1.as_shape::<Segment<N>>(), g2.as_shape::<Segment<N>>())
    {
        closest_points_internal::segment_against_segment(m1, s1, m2, s2, max_dist)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        closest_points_internal::plane_against_support_map(m1, p1, m2, s2, max_dist)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        closest_points_internal::support_map_against_plane(m1, s1, m2, p2, max_dist)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        let m12 = m1.inverse() * m2;
        let mut res =
            closest_points_internal::support_map_against_support_map(s1, &m12, s2, max_dist);
        res.transform(m1);
        res
    } else if let Some(c1) = g1.as_composite_shape() {
        closest_points_internal::composite_shape_against_shape(m1, c1, m2, g2, max_dist)
    } else if let Some(c2) = g2.as_composite_shape() {
        closest_points_internal::shape_against_composite_shape(m1, g1, m2, c2, max_dist)
    } else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
