use std::num::Float;
use nalgebra::na::Translation;
use nalgebra::na;
use ray::{Ray, RayCast, RayIntersection};
use geom::Ball;
use math::{Scalar, Vect, Matrix};

#[cfg(dim3)]
use nalgebra::na::Vec2;


#[cfg(dim3)]
fn ball_uv(normal: &Vect) -> Option<Vec2<Scalar>> {
    let two_pi: Scalar = Float::two_pi();
    let pi:     Scalar = Float::pi();
    let _0_5:   Scalar = na::cast(0.5);
    let uvx       = _0_5 + normal.z.atan2(normal.x) / two_pi;
    let uvy       = _0_5 - normal.y.asin() / pi;

    Some(Vec2::new(uvx, uvy))
}

impl RayCast for Ball {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray, solid: bool) -> Option<Scalar> {
        ball_toi_with_ray(na::zero(), self.radius(), ray, solid).val1()
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        let (inside, inter) = ball_toi_with_ray(na::zero(), self.radius(), ray, solid);
        inter.map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(&pos);

            RayIntersection::new(n, if inside { -normal } else { normal })
        })
    }

    #[cfg(dim3)]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        let (inside, inter) = ball_toi_with_ray(na::zero(), self.radius(), ray, solid);

        inter.map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(&pos);
            let uv     = ball_uv(&normal);

            RayIntersection::new_with_uvs(n, if inside { -normal } else { normal }, uv)
        })
    }

    #[inline]
    fn toi_with_transform_and_ray(&self, m: &Matrix, ray: &Ray, solid: bool) -> Option<Scalar> {
        ball_toi_with_ray(m.translation(), self.radius(), ray, solid).val1()
    }
}

/// Computes the time of impact of a ray on a ball.
#[inline]
pub fn ball_toi_with_ray(center: Vect, radius: Scalar, ray: &Ray, solid: bool) -> (bool, Option<Scalar>) {
    let dcenter = ray.orig - center;

    let b = na::dot(&dcenter, &ray.dir);
    let c = na::sqnorm(&dcenter) - radius * radius;

    if c > na::zero() && b > na::zero() {
        (false, None)
    }
    else {
        let delta = b * b - c;

        if delta < na::zero() {
            // no solution
            (false, None)
        }
        else {
            let t = -b - delta.sqrt();

            if t <= na::zero() {
                // orig inside of the ball
                if solid {
                    (true, Some(na::zero()))
                }
                else {
                    (true, Some(-b + delta.sqrt()))
                }
            }
            else {
                (false, Some(t))
            }
        }
    }
}
