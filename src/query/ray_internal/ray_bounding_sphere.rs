use alga::general::Id;

use query::{Ray, RayCast, RayIntersection};
use shape::Ball;
use bounding_volume::BoundingSphere;
use math::{Isometry, Point};

impl<N: Real> RayCast<P, M> for BoundingSphere<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<P>, solid: bool) -> Option<N> {
        let centered_ray = ray.translate_by(-m.transform_point(self.center()).coords);

        Ball::new(self.radius()).toi_with_ray(m, &centered_ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<Vector<N>>> {
        let centered_ray = ray.translate_by(-m.transform_point(self.center()).coords);

        Ball::new(self.radius()).toi_and_normal_with_ray(&Id::new(), &centered_ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<Vector<N>>> {
        let centered_ray = ray.translate_by(-m.transform_point(self.center()).coords);

        Ball::new(self.radius()).toi_and_normal_and_uv_with_ray(&Id::new(), &centered_ray, solid)
    }

    #[inline]
    fn intersects_ray(&self, m: &Isometry<N>, ray: &Ray<P>) -> bool {
        let centered_ray = ray.translate_by(-m.transform_point(self.center()).coords);

        Ball::new(self.radius()).intersects_ray(&Id::new(), &centered_ray)
    }
}
