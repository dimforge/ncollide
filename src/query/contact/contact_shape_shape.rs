use na::RealField;

use crate::math::{Isometry, Point};
use crate::query::{self, Contact};
use crate::shape::{Ball, Plane, Shape};

/// Computes one contact point between two shapes.
///
/// Returns `None` if the objects are separated by a distance greater than `prediction`.
pub fn contact<N: RealField>(
    m1: &Isometry<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    g2: &Shape<N>,
    prediction: N,
) -> Option<Contact<N>>
{
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        let p1 = Point::from(m1.translation.vector);
        let p2 = Point::from(m2.translation.vector);

        query::contact_ball_ball(&p1, b1, &p2, b2, prediction)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        query::contact_plane_support_map(m1, p1, m2, s2, prediction)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        query::contact_support_map_plane(m1, s1, m2, p2, prediction)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        query::contact_support_map_support_map(m1, s1, m2, s2, prediction)
    } else if let Some(c1) = g1.as_composite_shape() {
        query::contact_composite_shape_shape(m1, c1, m2, g2, prediction)
    } else if let Some(c2) = g2.as_composite_shape() {
        query::contact_shape_composite_shape(m1, g1, m2, c2, prediction)
    } else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}
