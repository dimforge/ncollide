use alga::general::Real;
use na;
#[cfg(feature = "dim3")]
use na::Point2;

#[cfg(feature = "dim3")]
use math::Vector;
use math::{Isometry, Point};
use query::{Ray, RayCast, RayIntersection};
use shape::{Ball, FeatureId};

#[cfg(feature = "dim3")]
#[inline]
fn ball_uv<N: Real>(normal: &Vector<N>) -> Point2<N> {
    let two_pi: N = Real::two_pi();
    let pi: N = Real::pi();
    let _0_5: N = na::convert(0.5f64);
    let uvx = _0_5 + normal[2].atan2(normal[0]) / two_pi;
    let uvy = _0_5 - normal[1].asin() / pi;

    Point2::new(uvx, uvy)
}

impl<N: Real> RayCast<N> for Ball<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, solid: bool) -> Option<N> {
        ball_toi_with_ray(
            &Point::from_coordinates(m.translation.vector),
            self.radius(),
            ray,
            solid,
        )
        .1
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let center = Point::from_coordinates(m.translation.vector);
        let (inside, inter) = ball_toi_with_ray(&center, self.radius(), ray, solid);

        inter.map(|n| {
            let pos = ray.origin + ray.dir * n - center;
            let normal = na::normalize(&pos);

            RayIntersection::new(n, if inside { -normal } else { normal }, FeatureId::Face(0))
        })
    }

    #[cfg(feature = "dim3")]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let center = Point::from_coordinates(m.translation.vector);
        let (inside, inter) = ball_toi_with_ray(&center, self.radius(), ray, solid);

        inter.map(|n| {
            let pos = ray.origin + ray.dir * n - center;
            let normal = na::normalize(&pos);
            let uv = ball_uv(&normal);

            RayIntersection::new_with_uvs(n, if inside { -normal } else { normal }, FeatureId::Face(0), Some(uv))
        })
    }
}

/// Computes the time of impact of a ray on a ball.
#[inline]
pub fn ball_toi_with_ray<N: Real>(
    center: &Point<N>,
    radius: N,
    ray: &Ray<N>,
    solid: bool,
) -> (bool, Option<N>)
{
    let dcenter = ray.origin - *center;

    let a = na::norm_squared(&ray.dir);
    let b = na::dot(&dcenter, &ray.dir);
    let c = na::norm_squared(&dcenter) - radius * radius;

    if c > na::zero() && b > na::zero() {
        (false, None)
    } else {
        let delta = b * b - a * c;

        if delta < na::zero() {
            // no solution
            (false, None)
        } else {
            let t = (-b - delta.sqrt()) / a;

            if t <= na::zero() {
                // origin inside of the ball
                if solid {
                    (true, Some(na::zero()))
                } else {
                    (true, Some((-b + delta.sqrt()) / a))
                }
            } else {
                (false, Some(t))
            }
        }
    }
}
