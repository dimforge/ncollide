use na::RealField;

use crate::interpolation::RigidMotion;
use crate::query::{self, TOI};
use crate::shape::{Ball, Shape};

/// Computes the smallest time of impact of two shapes under translational movement.
pub fn nonlinear_time_of_impact<N: RealField>(
    motion1: &(impl RigidMotion<N> + ?Sized),
    g1: &dyn Shape<N>,
    motion2: &(impl RigidMotion<N> + ?Sized),
    g2: &dyn Shape<N>,
    max_toi: N,
    target_distance: N,
) -> Option<TOI<N>>
{
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        query::nonlinear_time_of_impact_ball_ball(motion1, b1, motion2, b2, max_toi, target_distance)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        query::nonlinear_time_of_impact_support_map_support_map(motion1, s1, motion2, s2, max_toi, target_distance)
    } else if let Some(c1) = g1.as_composite_shape() {
        query::nonlinear_time_of_impact_composite_shape_shape(motion1, c1, motion2, g2, max_toi, target_distance)
    } else if let Some(c2) = g2.as_composite_shape() {
        query::nonlinear_time_of_impact_shape_composite_shape(motion1, g1, motion2, c2, max_toi, target_distance)
    /* } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
//        query::nonlinear_time_of_impact_plane_support_map(m1, vel1, p1, m2, vel2, s2)
        unimplemented!()
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
//        query::nonlinear_time_of_impact_support_map_plane(m1, vel1, s1, m2, vel2, p2)
        unimplemented!() */
    } else {
        eprintln!("No algorithm known to compute a contact point between the given pair of shapes.");
        None
    }
}