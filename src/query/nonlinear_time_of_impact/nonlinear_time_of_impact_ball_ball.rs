use na::RealField;

use crate::math::{Point, Vector};
use crate::query::{self, Ray};
use crate::shape::Ball;

/// Non-linear Time Of Impact of two balls under a rigid motion (translation + rotation).
#[inline]
pub fn nonlinear_time_of_impact_ball_ball<N: RealField>(
    center1: &Point<N>,
    vel1: &Vector<N>,
    b1: &Ball<N>,
    center2: &Point<N>,
    vel2: &Vector<N>,
    b2: &Ball<N>,
) -> Option<N>
{
    query::time_of_impact_ball_ball(center1, vel1, b1, center2, vel2, b2)
}
