use crate::math::{Point, Vector};
use crate::query::Contact;
use crate::shape::Ball;
use na::{self, RealField, Unit};

/// Contact between balls.
#[inline]
pub fn contact_ball_ball<N: RealField>(
    center1: &Point<N>,
    b1: &Ball<N>,
    center2: &Point<N>,
    b2: &Ball<N>,
    prediction: N,
) -> Option<Contact<N>> {
    let r1 = b1.radius;
    let r2 = b2.radius;
    let delta_pos = *center2 - *center1;
    let distance_squared = delta_pos.norm_squared();
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + prediction;

    if distance_squared < sum_radius_with_error * sum_radius_with_error {
        let normal = if !distance_squared.is_zero() {
            Unit::new_normalize(delta_pos)
        } else {
            Vector::x_axis()
        };

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
