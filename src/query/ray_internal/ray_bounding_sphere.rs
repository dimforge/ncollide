use bounding_volume::BoundingSphere;
use math::Isometry;
use na::Real;
use query::{Ray, RayCast, RayIntersection};
use shape::Ball;

impl<N: Real> RayCast<N> for BoundingSphere<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, solid: bool) -> Option<N> {
        let centered_ray = ray.translate_by(-(m * self.center()).coords);

        Ball::new(self.radius()).toi_with_ray(m, &centered_ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let centered_ray = ray.translate_by(-(m * self.center()).coords);

        Ball::new(self.radius()).toi_and_normal_with_ray(
            &Isometry::identity(),
            &centered_ray,
            solid,
        )
    }

    #[cfg(feature = "dim3")]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let centered_ray = ray.translate_by(-(m * self.center()).coords);

        Ball::new(self.radius()).toi_and_normal_and_uv_with_ray(
            &Isometry::identity(),
            &centered_ray,
            solid,
        )
    }

    #[inline]
    fn intersects_ray(&self, m: &Isometry<N>, ray: &Ray<N>) -> bool {
        let centered_ray = ray.translate_by(-(m * self.center()).coords);

        Ball::new(self.radius()).intersects_ray(&Isometry::identity(), &centered_ray)
    }
}
