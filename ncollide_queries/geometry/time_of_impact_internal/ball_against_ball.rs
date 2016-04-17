use na;
use math::{Point, Vector};
use entities::shape::Ball;
use ray::Ray;
use ray;

/// Time Of Impact of two balls under translational movement.
#[inline]
pub fn ball_against_ball<P>(center1: &P, vel1: &P::Vect, b1: &Ball<<P::Vect as Vector>::Scalar>,
                            center2: &P, vel2: &P::Vect, b2: &Ball<<P::Vect as Vector>::Scalar>)
                            -> Option<<P::Vect as Vector>::Scalar>
    where P: Point {
    let vel    = *vel1 - *vel2;
    let radius = b1.radius() + b2.radius();
    let center = *center1 + (-*center2.as_vector());

    ray::ball_toi_with_ray(&center, radius, &Ray::new(na::origin(), -vel), true).1
}
