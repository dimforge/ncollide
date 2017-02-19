use alga::general::Id;

use query::{Ray, RayCast, RayIntersection};
use shape::Ball;
use bounding_volume::BoundingSphere;
use math::{Point, Isometry};


impl<P: Point, M: Isometry<P>> RayCast<P, M> for BoundingSphere<P> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<P::Real> {
        let centered_ray = ray.translate_by(-m.transform_point(self.center()).coordinates());

        Ball::new(self.radius()).toi_with_ray(m, &centered_ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vector>> {
        let centered_ray = ray.translate_by(-m.transform_point(self.center()).coordinates());

        Ball::new(self.radius()).toi_and_normal_with_ray(&Id::new(), &centered_ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vector>> {
        let centered_ray = ray.translate_by(-m.transform_point(self.center()).coordinates());

        Ball::new(self.radius()).toi_and_normal_and_uv_with_ray(&Id::new(), &centered_ray, solid)
    }

    #[inline]
    fn intersects_ray(&self, m: &M, ray: &Ray<P>) -> bool {
        let centered_ray = ray.translate_by(-m.transform_point(self.center()).coordinates());

        Ball::new(self.radius()).intersects_ray(&Id::new(), &centered_ray)
    }
}
