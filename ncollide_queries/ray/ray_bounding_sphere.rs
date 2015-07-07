use na::{Identity, Transform, Rotate, Translate};
use ray::{Ray, RayCast, RayIntersection};
use entities::shape::Ball;
use entities::bounding_volume::BoundingSphere;
use math::{Scalar, Point, Vect};


impl<P, M> RayCast<P, M> for BoundingSphere<P>
    where P: Point,
          M: Transform<P> + Translate<P> + Rotate<P::Vect> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vect>::Scalar> {
        let centered_ray = Ray::new(ray.orig + (-*m.transform(self.center()).as_vec()), ray.dir);

        Ball::new(self.radius()).toi_with_ray(m, &centered_ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let centered_ray = Ray::new(ray.orig + (-*m.transform(self.center()).as_vec()), ray.dir);

        Ball::new(self.radius()).toi_and_normal_with_ray(&Identity::new(), &centered_ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let centered_ray = Ray::new(ray.orig + (-*m.transform(self.center()).as_vec()), ray.dir);

        Ball::new(self.radius()).toi_and_normal_and_uv_with_ray(&Identity::new(), &centered_ray, solid)
    }

    #[inline]
    fn intersects_ray(&self, m: &M, ray: &Ray<P>) -> bool {
        let centered_ray = Ray::new(ray.orig + (-*m.transform(self.center()).as_vec()), ray.dir);

        Ball::new(self.radius()).intersects_ray(&Identity::new(), &centered_ray)
    }
}
