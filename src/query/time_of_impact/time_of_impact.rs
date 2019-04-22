use na::RealField;

use crate::math::{Isometry, Point, Vector};
use crate::query;
use crate::shape::{Ball, Plane, Shape};

/// Computes the smallest time of impact of two shapes under translational movement.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn time_of_impact<N: RealField>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &Shape<N>,
) -> Option<N>
{
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        let p1 = Point::from(m1.translation.vector);
        let p2 = Point::from(m2.translation.vector);

        query::time_of_impact_ball_ball(&p1, vel1, b1, &p2, vel2, b2)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        query::time_of_impact_plane_support_map(m1, vel1, p1, m2, vel2, s2)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        query::time_of_impact_support_map_plane(m1, vel1, s1, m2, vel2, p2)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        query::time_of_impact_support_map_support_map(m1, vel1, s1, m2, vel2, s2)
    } else if let Some(c1) = g1.as_composite_shape() {
        query::time_of_impact_composite_shape_shape(m1, vel1, c1, m2, vel2, g2)
    } else if let Some(c2) = g2.as_composite_shape() {
        query::time_of_impact_shape_composite_shape(m1, vel1, g1, m2, vel2, c2)
    } else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
