use na::RealField;

use crate::math::{Isometry, Point};
use crate::query::{self, ClosestPoints};
use crate::shape::{Ball, Plane, Segment, Shape};

/// Computes the pair of closest points between two shapes.
///
/// Returns `None` if the objects are separated by a distance greater than `max_dist`.
pub fn closest_points<N: RealField>(
    m1: &Isometry<N>,
    g1: &dyn Shape<N>,
    m2: &Isometry<N>,
    g2: &dyn Shape<N>,
    max_dist: N,
) -> ClosestPoints<N> {
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        let p1 = Point::from(m1.translation.vector);
        let p2 = Point::from(m2.translation.vector);

        query::closest_points_ball_ball(&p1, b1, &p2, b2, max_dist)
    } else if let (Some(s1), Some(s2)) = (g1.as_shape::<Segment<N>>(), g2.as_shape::<Segment<N>>())
    {
        query::closest_points_segment_segment(m1, s1, m2, s2, max_dist)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        query::closest_points_plane_support_map(m1, p1, m2, s2, max_dist)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        query::closest_points_support_map_plane(m1, s1, m2, p2, max_dist)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        query::closest_points_support_map_support_map(m1, s1, m2, s2, max_dist)
    } else if let Some(c1) = g1.as_composite_shape() {
        query::closest_points_composite_shape_shape(m1, c1, m2, g2, max_dist)
    } else if let Some(c2) = g2.as_composite_shape() {
        query::closest_points_shape_composite_shape(m1, g1, m2, c2, max_dist)
    } else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
