use ray::{Ray, RayCast, RayIntersection};
use geom::Ball;
use bounding_volume::BoundingSphere;
use math::Scalar;

impl RayCast for BoundingSphere {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray, solid: bool) -> Option<Scalar> {
        let local_ray = Ray::new(ray.orig - *self.center(), ray.dir.clone());

        Ball::new(self.radius()).toi_with_ray(&local_ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        let local_ray = Ray::new(ray.orig - *self.center(), ray.dir.clone());

        Ball::new(self.radius()).toi_and_normal_with_ray(&local_ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        let local_ray = Ray::new(ray.orig - *self.center(), ray.dir.clone());

        Ball::new(self.radius()).toi_and_normal_and_uv_with_ray(&local_ray, solid)
    }

    #[inline]
    fn intersects_ray(&self, ray: &Ray) -> bool {
        let local_ray = Ray::new(ray.orig - *self.center(), ray.dir.clone());

        Ball::new(self.radius()).intersects_ray(&local_ray)
    }
}
