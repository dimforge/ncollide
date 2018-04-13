use bounding_volume::AABB;
use shape::Cuboid;
use query::{Ray, RayCast, RayIntersection};
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> RayCast<P, M> for Cuboid<P::Vector> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<P::Real> {
        let dl = P::from_coordinates(-*self.half_extents());
        let ur = P::from_coordinates(*self.half_extents());
        AABB::new(dl, ur).toi_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let dl = P::from_coordinates(-*self.half_extents());
        let ur = P::from_coordinates(*self.half_extents());
        AABB::new(dl, ur).toi_and_normal_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let dl = P::from_coordinates(-*self.half_extents());
        let ur = P::from_coordinates(*self.half_extents());
        AABB::new(dl, ur).toi_and_normal_and_uv_with_ray(m, ray, solid)
    }
}
