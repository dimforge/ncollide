use math::{Isometry, Point};
use shape::Shape;
use query::{Ray, RayCast, RayIntersection};

impl<N: Real> RayCast<P, M> for Shape<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<P>, solid: bool) -> Option<N> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<Vector<N>>> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_and_normal_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<Vector<N>>> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_and_normal_and_uv_with_ray(m, ray, solid)
    }

    #[inline]
    fn intersects_ray(&self, m: &Isometry<N>, ray: &Ray<P>) -> bool {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .intersects_ray(m, ray)
    }
}
