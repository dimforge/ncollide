use std::num::Algebraic;
use nalgebra::na::Translation;
use nalgebra::na;
use ray::{Ray, RayCast, RayCastWithTransform};
use geom::Ball;
use math::{N, V, M};

#[cfg(dim3)]
fn ball_uv(normal: &V) -> Option<(N, N, N)> {
    let two_pi: N = Real::two_pi();
    let pi:     N = Real::pi();
    let _0_5:   N = na::cast(0.5);
    let uvx       = _0_5 + normal.z.atan2(&normal.x) / two_pi;
    let uvy       = _0_5 - normal.y.asin() / pi;

    Some((uvx, uvy, na::zero()))
}

impl RayCast for Ball {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray) -> Option<N> {
        ball_toi_with_ray(na::zero(), self.radius(), ray)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<(N, V)> {
        ball_toi_with_ray(na::zero(), self.radius(), ray).map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(&pos);

            (n, normal)
        })
    }

    #[cfg(dim3)]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray) -> Option<(N, V, Option<(N, N, N)>)> {
        ball_toi_with_ray(na::zero(), self.radius(), ray).map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(&pos);
            let uv     = ball_uv(&normal);

            (n, normal, uv)
        })
    }
}

impl RayCastWithTransform for Ball {
    #[inline]
    fn toi_with_transform_and_ray(&self, m: &M, ray: &Ray) -> Option<N> {
        ball_toi_with_ray(m.translation(), self.radius(), ray)
    }
}

pub fn ball_toi_with_ray(center: V, radius: N, ray: &Ray) -> Option<N>{
    let dcenter = ray.orig - center;

    let b = na::dot(&dcenter, &ray.dir);
    let c = na::sqnorm(&dcenter) - radius * radius;

    if c > na::zero() && b > na::zero() {
        None
    }
    else {
        let delta = b * b - c;

        if delta < na::zero() {
            // no solution
            None
        }
        else {
            let t = -b - delta.sqrt();

            if t < na::zero() {
                // orig inside of the ball
                Some(na::zero())
            }
            else {
                Some(t)
            }
        }
    }
}
