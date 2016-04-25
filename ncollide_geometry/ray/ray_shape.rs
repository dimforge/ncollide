use math::{Point, Vector, Isometry};
use shape::Shape;
use ray::{RayCast, Ray, RayIntersection};

impl<P, M> RayCast<P, M> for Shape<P, M>
    where P: Point,
          M: Isometry<P> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vector>::Scalar> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        self.as_ray_cast()
            .expect("No RayCast implementation for the underlying shape.")
            .toi_and_normal_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
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
