use crate::math::Point;
use na::{self, Real};
use crate::query::ClosestPoints;
use crate::shape::Ball;

/// Proximity between balls.
#[inline]
pub fn ball_against_ball<N: Real>(
    center1: &Point<N>,
    b1: &Ball<N>,
    center2: &Point<N>,
    b2: &Ball<N>,
    margin: N,
) -> ClosestPoints<N>
{
    assert!(
        margin >= na::zero(),
        "The proximity margin must be positive or null."
    );

    let r1 = b1.radius();
    let r2 = b2.radius();
    let delta_pos = *center2 - *center1;
    let distance_squared = delta_pos.norm_squared();
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + margin;

    if distance_squared <= sum_radius_with_error * sum_radius_with_error {
        if distance_squared <= sum_radius * sum_radius {
            ClosestPoints::Intersecting
        } else {
            let normal = delta_pos.normalize();
            ClosestPoints::WithinMargin(*center1 + normal * r1, *center2 + normal * (-r2))
        }
    } else {
        ClosestPoints::Disjoint
    }
}
