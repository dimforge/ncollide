use na::{RealField, Unit};

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
    max_toi: N,
    target_distance: N,
) -> Option<TOI<N>>
{
    let vel = *vel1 - *vel2;
    let rsum = b1.radius() + b2.radius();
    let radius = rsum + target_distance;
    let center = *center1 + (-center2.coords);
    let ray = Ray::new(Point::origin(), -vel);

    if let (inside, Some(toi)) = query::ray_toi_with_ball(&center, radius, &ray, true) {
        if toi > max_toi {
            return None;
        }

        let dpt = ray.point_at(toi) - center;
        let normal1;
        let normal2;
        let witness1;
        let witness2;

        if radius.is_zero() {
            normal1 = Vector::x_axis();
            normal2 = Vector::x_axis();
            witness1 = Point::origin();
            witness2 = Point::origin();
        } else {
            normal1 = Unit::new_unchecked(dpt / radius);
            normal2 = -normal1;
            witness1 = Point::from(*normal1 * b1.radius());
            witness2 = Point::from(*normal2 * b2.radius());
        }

        let status = if inside && center.coords.norm_squared() < rsum * rsum {
            TOIStatus::Penetrating
        } else {
            TOIStatus::Converged
        };

        Some(TOI {
            toi,
            normal1,
            normal2,
            witness1,
            witness2,
            status,
        })
    } else {
        None
    }
}
