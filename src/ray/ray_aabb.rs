use std::util;
use nalgebra::na::{AlgebraicVecExt, Indexable};
use nalgebra::na;
use ray::{Ray, RayCast};
use bounding_volume;

impl<N: Primitive + Orderable + Algebraic,
     V: AlgebraicVecExt<N>>
RayCast<N, V> for bounding_volume::AABB<N, V> {
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        let mut tmin: N = na::zero();
        let mut tmax: N = Bounded::max_value();

        for i in range(0u, na::dim::<V>()) {
            if ray.dir.at(i).is_zero() {
                if ray.orig.at(i) < self.mins().at(i) || ray.orig.at(i) > self.maxs().at(i) {
                    return None
                }
            }
            else {
                let _1: N = na::one();
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

    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        let mut tmin: N = na::zero();
        let mut tmax: N = Bounded::max_value();
        let mut side = 0u;
        let mut diag = false;

        for i in range(0u, na::dim::<V>()) {
            if ray.dir.at(i).is_zero() {
                if ray.orig.at(i) < self.mins().at(i) || ray.orig.at(i) > self.maxs().at(i) {
                    return None
                }
            }
            else {
                let _1: N = na::one();
                let denom = _1 / ray.dir.at(i);
                let mut inter_with_near_plane = (self.mins().at(i) - ray.orig.at(i)) * denom;
                let mut inter_with_far_plane  = (self.maxs().at(i) - ray.orig.at(i)) * denom;

                if inter_with_near_plane > inter_with_far_plane {
                    util::swap(&mut inter_with_near_plane, &mut inter_with_far_plane)
                }

                if inter_with_near_plane > tmin {
                    tmin = inter_with_near_plane;
                    side = i;
                    diag = false;
                }
                else if inter_with_near_plane == tmin {
                    diag = true;
                }

                tmax = tmax.min(&inter_with_far_plane);

                if tmin > tmax {
                    return None;
                }
            }
        }

        if diag {
            Some((tmin, -na::normalize(&ray.dir)))
        }
        else {
            let mut normal: V = na::zero();
            normal.set(side, na::one());
            Some((tmin, normal))
        }
    }
}
