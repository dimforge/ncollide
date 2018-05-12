use na::Real;

use math::{Point, Vector};
use query::{ray_internal, Ray};
use shape::Ball;

/// Time Of Impact of two balls under translational movement.
#[inline]
pub fn ball_against_ball<N: Real>(
    center1: &Point<N>,
    vel1: &Vector<N>,
    b1: &Ball<N>,
    center2: &Point<N>,
    vel2: &Vector<N>,
    b2: &Ball<N>,
) -> Option<N> {
    let vel = *vel1 - *vel2;
    let radius = b1.radius() + b2.radius();
    let center = *center1 + (-center2.coords);

    ray_internal::ball_toi_with_ray(&center, radius, &Ray::new(Point::origin(), -vel), true).1
}
