use na::RealField;

use crate::math::{Isometry, Point};
use crate::query::{self, Proximity};
use crate::shape::{Ball, Plane, Shape};

/// Tests whether two shapes are in intersecting or separated by a distance smaller than `margin`.
pub fn proximity<N: RealField>(
    m1: &Isometry<N>,
    g1: &dyn Shape<N>,
    m2: &Isometry<N>,
    g2: &dyn Shape<N>,
    margin: N,
) -> Proximity
{
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        let p1 = Point::from(m1.translation.vector);
        let p2 = Point::from(m2.translation.vector);

        query::proximity_ball_ball(&p1, b1, &p2, b2, margin)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        query::proximity_plane_support_map(m1, p1, m2, s2, margin)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        query::proximity_support_map_plane(m1, s1, m2, p2, margin)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        query::proximity_support_map_support_map(m1, s1, m2, s2, margin)
    } else if let Some(c1) = g1.as_composite_shape() {
        query::proximity_composite_shape_shape(m1, c1, m2, g2, margin)
    } else if let Some(c2) = g2.as_composite_shape() {
        query::proximity_shape_composite_shape(m1, g1, m2, c2, margin)
    } else {
        panic!("No algorithm known to compute proximity between the given pair of shapes.")
    }
}
