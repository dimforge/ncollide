use na;
use ray::{Ray, RayCast, RayIntersection};
use geom::Plane;
use math::{Scalar, Point, Vect};

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray(center: &Point, normal: &Vect, ray: &Ray) -> Option<Scalar> {
    let dpos = center - ray.orig;
    let t    = na::dot(normal, &dpos) / na::dot(normal, &ray.dir);

    if t >= na::zero() {
        Some(t)
    }
    else {
        None
    }
}

impl RayCast for Plane {
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        let dpos = -ray.orig;

        let dot_normal_dpos = na::dot(&self.normal(), dpos.as_vec());

        if solid && dot_normal_dpos > na::zero() {
            // The ray is inside of the solid half-space.
            return Some(RayIntersection::new(na::zero(), na::zero()))
        }

        let t = dot_normal_dpos / na::dot(&self.normal(), &ray.dir);

        if t >= na::zero() {
            let n = if dot_normal_dpos > na::zero() { -self.normal() } else { self.normal() };

            Some(RayIntersection::new(t, n))
        }
        else {
            None
        }
    }
}
