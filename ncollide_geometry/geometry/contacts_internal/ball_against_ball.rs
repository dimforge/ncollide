use num::{Float, Zero};
use na;
use math::{Point, Vector};
use geometry::Contact;
use shape::Ball;

/// Contact between balls.
#[inline]
pub fn ball_against_ball<P>(center1: &P, b1: &Ball<<P::Vect as Vector>::Scalar>,
                            center2: &P, b2: &Ball<<P::Vect as Vector>::Scalar>,
                            prediction: <P::Vect as Vector>::Scalar)
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
            normal = na::canonical_basis_element(0).unwrap();
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
