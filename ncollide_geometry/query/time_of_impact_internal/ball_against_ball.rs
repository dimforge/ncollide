use math::Point;
use shape::Ball;
use query::{ray_internal, Ray};

/// Time Of Impact of two balls under translational movement.
#[inline]
pub fn ball_against_ball<P>(center1: &P, vel1: &P::Vector, b1: &Ball<P::Real>,
                            center2: &P, vel2: &P::Vector, b2: &Ball<P::Real>)
                            -> Option<P::Real>
    where P: Point {
    let vel    = *vel1 - *vel2;
    let radius = b1.radius() + b2.radius();
    let center = *center1 + (-center2.coordinates());

    ray_internal::ball_toi_with_ray(&center, radius, &Ray::new(P::origin(), -vel), true).1
}
