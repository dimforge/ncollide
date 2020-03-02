use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point};
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::Cuboid;
use na::RealField;

impl<N: RealField> RayCast<N> for Cuboid<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, max_toi: N, solid: bool) -> Option<N> {
        let dl = Point::from(-*self.half_extents());
        let ur = Point::from(*self.half_extents());
        AABB::new(dl, ur).toi_with_ray(m, ray, max_toi, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let dl = Point::from(-*self.half_extents());
        let ur = Point::from(*self.half_extents());
        AABB::new(dl, ur).toi_and_normal_with_ray(m, ray, max_toi, solid)
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
        let dl = Point::from(-*self.half_extents());
        let ur = Point::from(*self.half_extents());
        AABB::new(dl, ur).toi_and_normal_and_uv_with_ray(m, ray, max_toi, solid)
    }
}
