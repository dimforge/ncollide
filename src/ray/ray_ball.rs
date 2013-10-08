use std::num::{Zero, Algebraic};
use nalgebra::na::{AlgebraicVec, Translation, Rotate, Transform};
use ray::{Ray, RayCast, RayCastWithTransform};
use geom::Ball;

impl<N: Num + Algebraic + Ord + Clone,
     V: AlgebraicVec<N> + Clone>
RayCast<N, V> for Ball<N> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        toi_with_ray(Zero::zero(), self.radius(), ray)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        do toi_with_ray(Zero::zero(), self.radius(), ray).map_move |n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = pos.normalized();

            (n, normal)
        }
    }
}

impl<N: Num + Algebraic + Ord + Clone,
     V: AlgebraicVec<N> + Clone,
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

    let b = dcenter.dot(&ray.dir);
    let c = dcenter.sqnorm() - radius * radius;

    if c > Zero::zero() && b > Zero::zero() {
        None
    }
    else {
        let delta = b * b - c;

        if delta < Zero::zero() {
            // no solution
            None
        }
        else {
            let t = -b - delta.sqrt();

            if t < Zero::zero() {
                // orig inside of the ball
                Some(Zero::zero())
            }
            else {
                Some(t)
            }
        }
    }
}
