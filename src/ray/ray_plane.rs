use std::num::Zero;
use nalgebra::na;
use ray::{Ray, RayCast, RayCastWithTransform};
use geom::Plane;
use math::{N, V};

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray(center: &V,
                          normal: &V,
                          ray:    &Ray)
                          -> Option<N> {
    let dpos = center - ray.orig;

    let t = na::dot(normal, &dpos) / na::dot(normal, &ray.dir);

    if t >= Zero::zero() {
        Some(t)
    }
    else {
        None
    }
}

impl RayCast for Plane {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray) -> Option<N> {
        plane_toi_with_ray(&Zero::zero(), &self.normal(), ray)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<(N, V)> {
        plane_toi_with_ray(&Zero::zero(), &self.normal(), ray).map(|t| {
            (t, self.normal())
        })
    }
}

impl RayCastWithTransform for Plane { }
