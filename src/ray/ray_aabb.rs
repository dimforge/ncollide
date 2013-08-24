use std::util;
use std::num::{Zero, One};
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::vector::VecExt;
use ray::ray::{Ray, RayCast};
use bounding_volume::aabb;

impl<N: Primitive + Orderable + ToStr,
     V: VecExt<N> + ToStr>
RayCast<N, V> for aabb::AABB<N, V> {
    /// Computes the toi of a ray and this aabb.
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        let mut tmin = Zero::zero::<N>();
        let mut tmax = Bounded::max_value::<N>();

        for i in range(0u, Dim::dim::<V>()) {
            if ray.dir.at(i).is_zero() {
                if ray.orig.at(i) < self.mins().at(i) || ray.orig.at(i) > self.maxs().at(i) {
                    return None
                }
            }
            else {
                let denom = One::one::<N>() / ray.dir.at(i);
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
