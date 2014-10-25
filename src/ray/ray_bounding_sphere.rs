use na::{Transform, Rotate};
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use geom::Ball;
use bounding_volume::BoundingSphere;
use math::{Scalar, Point, Vect};


impl<N, P, V> LocalRayCast<N, P, V> for BoundingSphere<N, P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        let local_ray = Ray::new(ray.orig + (-*self.center().as_vec()), ray.dir.clone());

        Ball::new(self.radius()).toi_with_ray(&local_ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        let local_ray = Ray::new(ray.orig + (-*self.center().as_vec()), ray.dir.clone());

        Ball::new(self.radius()).toi_and_normal_with_ray(&local_ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        let local_ray = Ray::new(ray.orig + (-*self.center().as_vec()), ray.dir.clone());

        Ball::new(self.radius()).toi_and_normal_and_uv_with_ray(&local_ray, solid)
    }

    #[inline]
    fn intersects_ray(&self, ray: &Ray<P, V>) -> bool {
        let local_ray = Ray::new(ray.orig + (-*self.center().as_vec()), ray.dir.clone());

        Ball::new(self.radius()).intersects_ray(&local_ray)
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for BoundingSphere<N, P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}
