use std::num::{Zero, Algebraic};
use nalgebra::traits::translation::Translation;
use nalgebra::traits::vector::AlgebraicVec;
use ray::ray::{Ray, RayCast};
use geom::ball::Ball;

impl<N: Num + Algebraic + Ord + Clone,
     V: AlgebraicVec<N> + Clone,
     M: Translation<V>> RayCast<N, V, M> for Ball<N> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        let center  = m.translation();
        let dcenter = ray.orig - center;

        let b = dcenter.dot(&ray.dir);
        let c = dcenter.sqnorm() - self.radius() * self.radius();

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
}
