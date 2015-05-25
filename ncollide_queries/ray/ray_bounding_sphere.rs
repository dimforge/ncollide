use na::{Transform, Rotate};
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use entities::shape::Ball;
use entities::bounding_volume::BoundingSphere;
use math::{Scalar, Point, Vect};


impl<P> LocalRayCast<P> for BoundingSphere<P>
    where P: Point {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vect>::Scalar> {
        let local_ray = Ray::new(ray.orig + (-*self.center().as_vec()), ray.dir.clone());

        Ball::new(self.radius()).toi_with_ray(&local_ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let local_ray = Ray::new(ray.orig + (-*self.center().as_vec()), ray.dir.clone());

        Ball::new(self.radius()).toi_and_normal_with_ray(&local_ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let local_ray = Ray::new(ray.orig + (-*self.center().as_vec()), ray.dir.clone());

        Ball::new(self.radius()).toi_and_normal_and_uv_with_ray(&local_ray, solid)
    }

    #[inline]
    fn intersects_ray(&self, ray: &Ray<P>) -> bool {
        let local_ray = Ray::new(ray.orig + (-*self.center().as_vec()), ray.dir.clone());

        Ball::new(self.radius()).intersects_ray(&local_ray)
    }
}

impl<P, M> RayCast<P, M> for BoundingSphere<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
}
