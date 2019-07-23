use na::RealField;

use crate::math::{Point, Vector, Isometry};
use crate::interpolation::RigidMotion;
use crate::query::{self, Ray, NonlinearTOI, ClosestPoints};
use crate::shape::Ball;

/// Non-linear Time Of Impact of two balls under a rigid motion (translation + rotation).
#[inline]
pub fn nonlinear_time_of_impact_ball_ball<N: RealField>(
    motion1: &(impl RigidMotion<N> + ?Sized),
    b1: &Ball<N>,
    motion2: &(impl RigidMotion<N> + ?Sized),
    b2: &Ball<N>,
    max_toi: N,
    target_distance: N,
) -> Option<NonlinearTOI<N>>
{
    fn closest_points<N: RealField>(m1: &Isometry<N>, g1: &Ball<N>, m2: &Isometry<N>, g2: &Ball<N>, prediction: N) -> ClosestPoints<N> {
        query::closest_points_ball_ball(
            &m1.translation.vector.into(), g1,
            &m2.translation.vector.into(), g2,
            prediction)
    }

    query::nonlinear_time_of_impact_support_map_support_map_with_closest_points_function(
        motion1, b1, motion2,
        b2, max_toi, target_distance,
        closest_points
    )
}
