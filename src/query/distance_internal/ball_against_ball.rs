use crate::math::Point;
use na::{self, Real};
use crate::shape::Ball;

/// Distance between balls.
#[inline]
pub fn ball_against_ball<N: Real>(
    center1: &Point<N>,
    b1: &Ball<N>,
    center2: &Point<N>,
    b2: &Ball<N>,
) -> N
{
    let r1 = b1.radius();
    let r2 = b2.radius();
    let delta_pos = *center2 - *center1;
    let distance_squared = delta_pos.norm_squared();
    let sum_radius = r1 + r2;

    if distance_squared <= sum_radius * sum_radius {
        na::zero()
    } else {
        distance_squared.sqrt() - sum_radius
    }
}
