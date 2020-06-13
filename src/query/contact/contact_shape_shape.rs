use na::RealField;

use crate::math::{Isometry, Point};
use crate::query::{self, Contact};
use crate::shape::{Ball, Plane, Shape};

/// Computes one contact point between two shapes.
///
/// Returns `None` if the objects are separated by a distance greater than `prediction`.
pub fn contact<N: RealField>(
    m1: &Isometry<N>,
    g1: &dyn Shape<N>,
    m2: &Isometry<N>,
    g2: &dyn Shape<N>,
    prediction: N,
) -> Option<Contact<N>> {
    let ball1 = g1.as_shape::<Ball<N>>();
    let ball2 = g2.as_shape::<Ball<N>>();

    if let (Some(b1), Some(b2)) = (ball1, ball2) {
        let p1 = Point::from(m1.translation.vector);
        let p2 = Point::from(m2.translation.vector);

        query::contact_ball_ball(&p1, b1, &p2, b2, prediction)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        query::contact_plane_support_map(m1, p1, m2, s2, prediction)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        query::contact_support_map_plane(m1, s1, m2, p2, prediction)
    } else if let (Some(b1), (Some(_), Some(_))) =
        (ball1, (g2.as_convex_polyhedron(), g2.as_point_query()))
    {
        let p1 = Point::from(m1.translation.vector);
        query::contact_ball_convex_polyhedron(&p1, b1, m2, g2, prediction)
    } else if let ((Some(_), Some(_)), Some(b2)) =
        ((g1.as_convex_polyhedron(), g1.as_point_query()), ball2)
    {
        let p2 = Point::from(m2.translation.vector);
        query::contact_convex_polyhedron_ball(m1, g1, &p2, b2, prediction)
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
