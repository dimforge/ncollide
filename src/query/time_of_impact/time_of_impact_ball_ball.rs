use na::RealField;

use crate::math::{Point, Vector};
use crate::query::{self, Ray, TOI, TOIStatus};
use crate::shape::Ball;

/// Time Of Impact of two balls under translational movement.
#[inline]
pub fn time_of_impact_ball_ball<N: RealField>(
    center1: &Point<N>,
    vel1: &Vector<N>,
    b1: &Ball<N>,
    center2: &Point<N>,
    vel2: &Vector<N>,
    b2: &Ball<N>,
    distance: N,
) -> Option<TOI<N>>
{
    let vel = *vel1 - *vel2;
    let rsum = b1.radius() + b2.radius();
    let radius = rsum + distance;
    let center = *center1 + (-center2.coords);
    let ray = Ray::new(Point::origin(), -vel);

    if let (inside, Some(toi)) = query::ray_toi_with_ball(&center, radius, &ray, true) {
        let dpt = ray.point_at(toi) - center;
        let witness1;
        let witness2;

        if radius.is_zero() {
            witness1 = Point::origin();
            witness2 = Point::origin();
        } else {
            witness1 = Point::from(dpt) * (b1.radius() / radius);
            witness2 = Point::from(dpt) * (-b2.radius() / radius);
        }

        let status = if inside && center.coords.norm_squared() < rsum * rsum {
            TOIStatus::Penetrating
        } else {
            TOIStatus::Converged
        };

        Some(TOI {
            toi,
            witness1,
            witness2,
            status,
        })
    } else {
        None
    }
}
