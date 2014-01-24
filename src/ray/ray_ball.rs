use std::num::Real;
use nalgebra::na::Translation;
use nalgebra::na;
use ray::{Ray, RayCast, RayIntersection};
use geom::Ball;
use math::{N, V, M};

#[cfg(dim3)]
use nalgebra::na::Vec3;


#[cfg(dim3)]
fn ball_uv(normal: &V) -> Option<V> {
    let two_pi: N = Real::two_pi();
    let pi:     N = Real::pi();
    let _0_5:   N = na::cast(0.5);
    let uvx       = _0_5 + normal.z.atan2(&normal.x) / two_pi;
    let uvy       = _0_5 - normal.y.asin() / pi;

    Some(Vec3::new(uvx, uvy, na::zero())) // XXX: is it a good hard-code the type Vec3 ?
}

impl RayCast for Ball {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray) -> Option<N> {
        ball_toi_with_ray(na::zero(), self.radius(), ray)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<RayIntersection> {
        ball_toi_with_ray(na::zero(), self.radius(), ray).map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(&pos);

            RayIntersection::new(n, normal)
        })
    }

    #[cfg(dim3)]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray) -> Option<RayIntersection> {
        ball_toi_with_ray(na::zero(), self.radius(), ray).map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(&pos);
            let uv     = ball_uv(&normal);

            RayIntersection::new_with_uvs(n, normal, uv)
        })
    }

    #[inline]
    fn toi_with_transform_and_ray(&self, m: &M, ray: &Ray) -> Option<N> {
        ball_toi_with_ray(m.translation(), self.radius(), ray)
    }
}

/// Computes the time of impact of a ray on a ball.
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
