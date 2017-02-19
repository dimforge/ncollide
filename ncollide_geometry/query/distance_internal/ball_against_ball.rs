use alga::general::Real;
use na;
use math::Point;
use shape::Ball;

/// Distance between balls.
#[inline]
pub fn ball_against_ball<P>(center1: &P, b1: &Ball<P::Real>,
                            center2: &P, b2: &Ball<P::Real>)
                            -> P::Real
    where P: Point {
    let r1         = b1.radius();
    let r2         = b2.radius();
    let delta_pos  = *center2 - *center1;
    let distance_squared = na::norm_squared(&delta_pos);
    let sum_radius = r1 + r2;

    if distance_squared <= sum_radius * sum_radius {
        na::zero()
    }
    else {
        distance_squared.sqrt() - sum_radius
    }
}
