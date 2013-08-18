use std::num::Zero;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::vector::Vec;
use ray::ray::{Ray, RayCast};
use geom::plane::Plane;

impl<N: Num + Ord,
     V: Vec<N> + Clone,
     M: Rotate<V> + Translation<V>>
RayCast<N, V, M> for Plane<N, V> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        let plane_normal = m.rotate(&self.normal());
        let plane_center = m.translation();

        let dpos = plane_center - ray.orig;

        let t = plane_normal.dot(&dpos) / plane_normal.dot(&ray.dir);

        if t >= Zero::zero() {
            Some(t)
        }
        else {
            None
        }
    }

    #[inline]
    fn intersects_ray(&self, m: &M, ray: &Ray<V>) -> bool {
        let plane_normal = m.rotate(&self.normal());
        let plane_center = m.translation();

        let dpos = plane_center - ray.orig;

        let h = plane_normal.dot(&dpos);
        let k = plane_normal.dot(&ray.dir);

        (h > Zero::zero() && k > Zero::zero()) ||
        (h < Zero::zero() && k < Zero::zero())
    }
}
