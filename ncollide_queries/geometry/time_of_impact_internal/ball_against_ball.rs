use na;
use math::{Scalar, Point, Vect};
use entities::shape::Ball;
use ray::Ray;
use ray;

/// Time Of Impact of two balls under translational movement.
#[inline]
pub fn ball_against_ball<N, P, V>(center1: &P, vel1: &V, b1: &Ball<N>,
                                  center2: &P, vel2: &V, b2: &Ball<N>)
                                  -> Option<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let vel    = *vel1 - *vel2;
    let radius = b1.radius() + b2.radius();
    let center = *center1 + (-*center2.as_vec());

    ray::ball_toi_with_ray(center, radius, &Ray::new(na::orig(), -vel), true).1
}
