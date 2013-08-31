use std::num::Zero;
use nalgebra::mat::{Rotate, Transform};
use nalgebra::vec::Vec;
use ray::ray::{Ray, RayCast, RayCastWithTransform};
use geom::plane::Plane;

/// Computes the toi of a ray with a plane described by its center and normal.
#[inline]
pub fn plane_toi_with_ray<N: Num + Ord,
                          V: Vec<N> + Clone>(
                          center: &V,
                          normal: &V,
                          ray:    &Ray<V>)
                          -> Option<N> {
    let dpos = center - ray.orig;

    let t = normal.dot(&dpos) / normal.dot(&ray.dir);

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
}

impl<N: Num + Ord, V: Vec<N> + Clone, M: Rotate<V> + Transform<V>>
RayCastWithTransform<N, V, M> for Plane<N, V>;
