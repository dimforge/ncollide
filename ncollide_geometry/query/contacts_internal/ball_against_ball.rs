use num::Zero;

use alga::general::Real;
use alga::linear::FiniteDimVectorSpace;
use na;
use math::Point;
use query::Contact;
use shape::Ball;

/// Contact between balls.
#[inline]
pub fn ball_against_ball<P>(center1: &P, b1: &Ball<P::Real>,
                            center2: &P, b2: &Ball<P::Real>,
                            prediction: P::Real)
                            -> Option<Contact<P>>
    where P: Point {
    let r1         = b1.radius();
    let r2         = b2.radius();
    let delta_pos  = *center2 - *center1;
    let distance_squared     = na::norm_squared(&delta_pos);
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + prediction;

    if distance_squared < sum_radius_with_error * sum_radius_with_error {
        let mut normal = na::normalize(&delta_pos);

        if distance_squared.is_zero() {
            normal = P::Vector::canonical_basis_element(0);
        }

        Some(Contact::new(
                *center1 + normal * r1,
                *center2 + (-normal * r2),
                normal,
                (sum_radius - distance_squared.sqrt())))
    }
    else {
        None
    }
}
