use na::{Transform, Rotate};
use na;
use entities::bounding_volume::AABB;
use entities::shape::Cuboid;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use math::{Scalar, Point, Vect};


impl<P> LocalRayCast<P> for Cuboid<P::Vect>
    where P: Point {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vect>::Scalar> {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).toi_with_ray(ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).toi_and_normal_with_ray(ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).toi_and_normal_and_uv_with_ray(ray, solid)
    }
}

impl<P, M> RayCast<P, M> for Cuboid<P::Vect>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
}
