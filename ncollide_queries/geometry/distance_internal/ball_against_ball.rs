use na;
use math::{Scalar, Point, Vect};
use entities::shape::Ball;

/// Distance between balls.
#[inline]
pub fn ball_against_ball<N, P, V>(center1: &P, b1: &Ball<N>, center2: &P, b2: &Ball<N>) -> N
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let r1         = b1.radius();
    let r2         = b2.radius();
    let delta_pos  = *center2 - *center1;
    let sqdist     = na::sqnorm(&delta_pos);
    let sum_radius = r1 + r2;

    if sqdist <= sum_radius * sum_radius {
        na::zero()
    }
    else {
        sqdist.sqrt() - sum_radius
    }
}
