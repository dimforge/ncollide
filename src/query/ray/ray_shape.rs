use crate::math::Isometry;
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::Shape;
use na::RealField;

impl<N: RealField> RayCast<N> for dyn Shape<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, max_toi: N, solid: bool) -> Option<N> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_with_ray(m, ray, max_toi, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_and_normal_with_ray(m, ray, max_toi, solid)
    }

    #[cfg(feature = "dim3")]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_and_normal_and_uv_with_ray(m, ray, max_toi, solid)
    }

    #[inline]
    fn intersects_ray(&self, m: &Isometry<N>, ray: &Ray<N>, max_toi: N) -> bool {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .intersects_ray(m, ray, max_toi)
    }
}
