use std::num::Zero;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::vector::Vec;
use ray::ray::{Ray, RayCast};
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

impl<N: Num + Ord,
     V: Vec<N> + Clone,
     M: Rotate<V> + Translation<V>>
RayCast<N, V, M> for Plane<N, V> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        let plane_normal = m.rotate(&self.normal());
        let plane_center = m.translation();

        plane_toi_with_ray(&plane_center, &plane_normal, ray)
    }
}
