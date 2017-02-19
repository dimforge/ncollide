use math::{Point, Isometry};
use shape::Shape;
use query::{RayCast, Ray, RayIntersection};

impl<P: Point, M: Isometry<P>> RayCast<P, M> for Shape<P, M> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<P::Real> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vector>> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_and_normal_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vector>> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_and_normal_and_uv_with_ray(m, ray, solid)
    }

    #[inline]
    fn intersects_ray(&self, m: &M, ray: &Ray<P>) -> bool {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .intersects_ray(m, ray)
    }
}
