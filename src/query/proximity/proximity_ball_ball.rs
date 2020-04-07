use crate::math::Point;
use crate::query::Proximity;
use crate::shape::Ball;
use na::{self, RealField};

/// Proximity between balls.
#[inline]
pub fn proximity_ball_ball<N: RealField>(
    center1: &Point<N>,
    b1: &Ball<N>,
    center2: &Point<N>,
    b2: &Ball<N>,
    margin: N,
) -> Proximity {
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
            Proximity::Intersecting
        } else {
            Proximity::WithinMargin
        }
    } else {
        Proximity::Disjoint
    }
}
