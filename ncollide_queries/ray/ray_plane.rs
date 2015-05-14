use na::{Transform, Rotate};
use na;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use entities::shape::Plane;
use math::{Scalar, Point, Vect};

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray<P>(center: &P, normal: &P::Vect, ray: &Ray<P>) -> Option<<P::Vect as Vect>::Scalar>
    where P: Point {
    let dpos = *center - ray.orig;
    let t    = na::dot(normal, &dpos) / na::dot(normal, &ray.dir);

    if t >= na::zero() {
        Some(t)
    }
    else {
        None
    }
}

impl<P> LocalRayCast<P> for Plane<P::Vect>
    where P: Point {
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let dpos = -ray.orig;

        let dot_normal_dpos = na::dot(self.normal(), dpos.as_vec());

        if solid && dot_normal_dpos > na::zero() {
            // The ray is inside of the solid half-space.
            return Some(RayIntersection::new(na::zero(), na::zero()))
        }

        let t = dot_normal_dpos / na::dot(self.normal(), &ray.dir);

        if t >= na::zero() {
            let n = if dot_normal_dpos > na::zero() { -*self.normal() } else { self.normal().clone() };

            Some(RayIntersection::new(t, n))
        }
        else {
            None
        }
    }
}

impl<P, M> RayCast<P, M> for Plane<P::Vect>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
}
