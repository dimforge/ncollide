use na;
use math::{Scalar, Point, Vect};
use geometry::Contact;
use entities::shape::Ball;

/// Contact between balls.
#[inline]
pub fn ball_against_ball<N, P, V>(center1: &P, b1: &Ball<N>,
                                  center2: &P, b2: &Ball<N>,
                                  prediction: N)
                                  -> Option<Contact<N, P, V>>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let r1         = b1.radius();
    let r2         = b2.radius();
    let delta_pos  = *center2 - *center1;
    let sqdist     = na::sqnorm(&delta_pos);
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + prediction;

    if sqdist < sum_radius_with_error * sum_radius_with_error {
        let mut normal = na::normalize(&delta_pos);

        if sqdist.is_zero() {
            normal = na::canonical_basis_element(0).unwrap();
        }

        Some(Contact::new(
                *center1 + normal * r1,
                *center2 + (-normal * r2),
                normal,
                (sum_radius - sqdist.sqrt())))
    }
    else {
        None
    }
}
