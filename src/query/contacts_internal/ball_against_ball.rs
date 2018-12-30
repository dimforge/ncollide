use crate::math::{Point, Vector};
use na::{self, Real, Unit};
use crate::query::Contact;
use crate::shape::Ball;

/// Contact between balls.
#[inline]
pub fn ball_against_ball<N: Real>(
    center1: &Point<N>,
    b1: &Ball<N>,
    center2: &Point<N>,
    b2: &Ball<N>,
    prediction: N,
) -> Option<Contact<N>>
{
    let r1 = b1.radius();
    let r2 = b2.radius();
    let delta_pos = *center2 - *center1;
    let distance_squared = na::norm_squared(&delta_pos);
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + prediction;

    if distance_squared < sum_radius_with_error * sum_radius_with_error {
        let mut normal = Unit::new_normalize(delta_pos);

        if distance_squared.is_zero() {
            normal = Vector::x_axis();
        }

        Some(Contact::new(
            *center1 + *normal * r1,
            *center2 + *normal * (-r2),
            normal,
            sum_radius - distance_squared.sqrt(),
        ))
    } else {
        None
    }
}
