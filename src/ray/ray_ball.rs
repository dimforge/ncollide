use std::num::Algebraic;
use nalgebra::na::{AlgebraicVec, AlgebraicVecExt, Translation, Rotate, Transform, Cast};
use nalgebra::na;
use ray::{Ray, RayCast, RayCastWithTransform};
use geom::Ball;

fn ball_uv<N: Num + Algebraic + Ord + Clone + Trigonometric + Real + Cast<f32>,
           V: AlgebraicVecExt<N> + Clone>(
           normal: &V)
           -> Option<(N, N, N)> {
    if na::dim::<V>() == 3 {
        let two_pi: N = Real::two_pi();
        let pi:     N = Real::pi();
        let _0_5:   N = na::cast(0.5);
        let uvx       = _0_5 + normal.at(2).atan2(&normal.at(0)) / two_pi;
        let uvy       = _0_5 - normal.at(1).asin() / pi;

        Some((uvx, uvy, na::zero()))
    }
    else {
        None
    }
}

impl<N: Num + Algebraic + Ord + Clone + Trigonometric + Real + Cast<f32>,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Ball<N> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        toi_with_ray(na::zero(), self.radius(), ray)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        toi_with_ray(na::zero(), self.radius(), ray).map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(&pos);

            (n, normal)
        })
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<V>) -> Option<(N, V, Option<(N, N, N)>)> {
        toi_with_ray(na::zero(), self.radius(), ray).map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(&pos);
            let uv     = ball_uv(&normal);

            (n, normal, uv)
        })
    }
}

impl<N: Num + Algebraic + Ord + Clone + Trigonometric + Real + Cast<f32>,
     V: AlgebraicVecExt<N> + Clone,
     M: Translation<V> + Rotate<V> + Transform<V>>
RayCastWithTransform<N, V, M> for Ball<N> {
    #[inline]
    fn toi_with_transform_and_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        toi_with_ray(m.translation(), self.radius(), ray)
    }
}

fn toi_with_ray<N: Num + Algebraic + Ord + Clone,
                V: AlgebraicVec<N> + Clone>(
                center: V,
                radius: N,
                ray:    &Ray<V>)
                -> Option<N>{
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
