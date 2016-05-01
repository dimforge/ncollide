use na::{self, Transform, Rotate};
use bounding_volume::AABB;
use shape::Cuboid;
use query::{Ray, RayCast, RayIntersection};
use math::{Point, Vector};


impl<P, M> RayCast<P, M> for Cuboid<P::Vect>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vector>::Scalar> {
        let dl = na::origin::<P>() + (-*self.half_extents());
        let ur = na::origin::<P>() + *self.half_extents();
        AABB::new(dl, ur).toi_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let dl = na::origin::<P>() + (-*self.half_extents());
        let ur = na::origin::<P>() + *self.half_extents();
        AABB::new(dl, ur).toi_and_normal_with_ray(m, ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let dl = na::origin::<P>() + (-*self.half_extents());
        let ur = na::origin::<P>() + *self.half_extents();
        AABB::new(dl, ur).toi_and_normal_and_uv_with_ray(m, ray, solid)
    }
}
