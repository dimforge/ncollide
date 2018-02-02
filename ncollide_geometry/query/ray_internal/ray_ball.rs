use alga::general::Real;
use na::{self, Point2};

use query::{Ray, RayCast, RayIntersection};
use shape::Ball;
use math::{Isometry, Point, Vector};

#[inline]
fn ball_uv<V: Vector>(normal: &V) -> Option<Point2<V::Real>> {
    if na::dimension::<V>() == 3 {
        let two_pi: V::Real = Real::two_pi();
        let pi: V::Real = Real::pi();
        let _0_5: V::Real = na::convert(0.5f64);
        let uvx = _0_5 + normal[2].atan2(normal[0]) / two_pi;
        let uvy = _0_5 - normal[1].asin() / pi;

        Some(Point2::new(uvx, uvy))
    } else {
        None
    }
}

impl<P: Point, M: Isometry<P>> RayCast<P, M> for Ball<P::Real> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<P::Real> {
        ball_toi_with_ray(&m.translate_point(&P::origin()), self.radius(), ray, solid).1
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let center = m.translate_point(&P::origin());
        let (inside, inter) = ball_toi_with_ray(&center, self.radius(), ray, solid);

        inter.map(|n| {
            let pos = ray.origin + ray.dir * n - center;
            let normal = na::normalize(&pos);

            RayIntersection::new(n, if inside { -normal } else { normal })
        })
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let center = m.translate_point(&P::origin());
        let (inside, inter) = ball_toi_with_ray(&center, self.radius(), ray, solid);

        inter.map(|n| {
            let pos = ray.origin + ray.dir * n - center;
            let normal = na::normalize(&pos);
            let uv = ball_uv(&normal);

            RayIntersection::new_with_uvs(n, if inside { -normal } else { normal }, uv)
        })
    }
}

/// Computes the time of impact of a ray on a ball.
#[inline]
pub fn ball_toi_with_ray<P>(
    center: &P,
    radius: P::Real,
    ray: &Ray<P>,
    solid: bool,
) -> (bool, Option<P::Real>)
where
    P: Point,
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
