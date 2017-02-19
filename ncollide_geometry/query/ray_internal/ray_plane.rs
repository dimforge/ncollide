use na;

use query::{Ray, RayCast, RayIntersection};
use shape::Plane;
use math::{Point, Isometry};

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray<P: Point>(center: &P, normal: &P::Vector, ray: &Ray<P>) -> Option<P::Real> {
    let dpos = *center - ray.origin;
    let t    = na::dot(normal, &dpos) / na::dot(normal, &ray.dir);

    if t >= na::zero() {
        Some(t)
    }
    else {
        None
    }
}

impl<P: Point, M: Isometry<P>> RayCast<P, M> for Plane<P::Vector> {
    #[inline]
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vector>> {
        let ls_ray = ray.inverse_transform_by(m);

        let dpos = -ls_ray.origin;

        let dot_normal_dpos = na::dot(self.normal(), &dpos.coordinates());

        if solid && dot_normal_dpos > na::zero() {
            // The ray is inside of the solid half-space.
            return Some(RayIntersection::new(na::zero(), na::zero()))
        }

        let t = dot_normal_dpos / na::dot(self.normal(), &ls_ray.dir);

        if t >= na::zero() {
            let n = if dot_normal_dpos > na::zero() { -*self.normal() } else { *self.normal() };

            Some(RayIntersection::new(t, m.rotate_vector(&n)))
        }
        else {
            None
        }
    }
}
