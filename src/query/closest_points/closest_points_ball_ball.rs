use crate::math::Point;
use crate::query::ClosestPoints;
use crate::shape::Ball;
use na::{self, RealField};

/// Proximity between balls.
#[inline]
pub fn closest_points_ball_ball<N: RealField>(
    center1: &Point<N>,
    b1: &Ball<N>,
    center2: &Point<N>,
    b2: &Ball<N>,
    margin: N,
) -> ClosestPoints<N> {
    assert!(
        margin >= na::zero(),
        "The proximity margin must be positive or null."
    );

    let r1 = b1.radius;
    let r2 = b2.radius;
    let delta_pos = *center2 - *center1;
    let distance = delta_pos.norm();
    let sum_radius = r1 + r2;

    if distance - margin <= sum_radius {
        if distance <= sum_radius {
            ClosestPoints::Intersecting
        } else {
            let normal = delta_pos.normalize();
            ClosestPoints::WithinMargin(*center1 + normal * r1, *center2 + normal * (-r2))
        }
    } else {
        ClosestPoints::Disjoint
    }
}
