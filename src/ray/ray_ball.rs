use std::num::{Zero, Algebraic};
use nalgebra::traits::translation::Translation;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::vector::AlgebraicVec;
use ray::ray::{Ray, RayCast, RayCastWithTransform};
use geom::ball::Ball;

impl<N: Num + Algebraic + Ord + Clone,
     V: AlgebraicVec<N> + Clone>
RayCast<N, V> for Ball<N> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        toi_with_ray(Zero::zero(), self.radius(), ray)
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
