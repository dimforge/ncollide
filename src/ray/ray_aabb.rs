use std::util;
use std::num::{Zero, One};
use nalgebra::vec::{VecExt, Indexable, Dim};
use ray::ray::{Ray, RayCast};
use bounding_volume::aabb;

impl<N: Primitive + Orderable + ToStr,
     V: VecExt<N> + ToStr>
RayCast<N, V> for aabb::AABB<N, V> {
    /// Computes the toi of a ray and this aabb.
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        let mut tmin: N = Zero::zero();
        let mut tmax: N = Bounded::max_value();

        for i in range(0u, Dim::dim(None::<V>)) {
            if ray.dir.at(i).is_zero() {
                if ray.orig.at(i) < self.mins().at(i) || ray.orig.at(i) > self.maxs().at(i) {
                    return None
                }
            }
            else {
                let _1: N = One::one();
                let denom = _1 / ray.dir.at(i);
                let mut inter_with_near_plane = (self.mins().at(i) - ray.orig.at(i)) * denom;
                let mut inter_with_far_plane  = (self.maxs().at(i) - ray.orig.at(i)) * denom;

                if inter_with_near_plane > inter_with_far_plane {
                    util::swap(&mut inter_with_near_plane, &mut inter_with_far_plane)
                }

                tmin = tmin.max(&inter_with_near_plane);
                tmax = tmax.min(&inter_with_far_plane);

                if tmin > tmax {
                    return None;
                }
            }
        }

        Some(tmin)
    }
}
