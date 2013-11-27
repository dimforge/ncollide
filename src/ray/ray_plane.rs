use std::num::Zero;
use nalgebra::na::{Rotate, Transform, Vec};
use nalgebra::na;
use ray::{Ray, RayCast, RayCastWithTransform};
use geom::Plane;

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray<N: Num + Ord,
                          V: Vec<N> + Clone>(
                          center: &V,
                          normal: &V,
                          ray:    &Ray<V>)
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

impl<N: Num + Ord, V: Vec<N> + Clone>
RayCast<N, V> for Plane<N, V> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        plane_toi_with_ray(&Zero::zero(), &self.normal(), ray)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        plane_toi_with_ray(&Zero::zero(), &self.normal(), ray).map(|n| {
            (n, self.normal())
        })
    }
}

impl<N: Num + Ord, V: Vec<N> + Clone, M: Rotate<V> + Transform<V>>
RayCastWithTransform<N, V, M> for Plane<N, V> { }
