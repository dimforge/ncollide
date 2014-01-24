use std::num::{Zero, Bounded};
use std::util;
use nalgebra::na::Indexable;
use nalgebra::na;
use ray::{Ray, RayCast, RayIntersection};
use bounding_volume::AABB;
use math::{N, V};

#[cfg(dim3)]
use nalgebra::na::Vec3;

impl RayCast for AABB {
    fn toi_with_ray(&self, ray: &Ray) -> Option<N> {
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

    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<RayIntersection> {
        ray_aabb(self, ray).map(|(t, n, _)| RayIntersection::new(t, n))
    }

    #[cfg(dim3)]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray) -> Option<RayIntersection> {
        ray_aabb(self, ray).map(|(t, n, s)| {
            let pt  = ray.orig + ray.dir * t;
            let lpt = (pt - *self.mins()) / (self.maxs() - *self.mins());
            let id  = s.abs();

            if id == 1 {
                RayIntersection::new_with_uvs(t, n, Some(Vec3::new(lpt.y.clone(), lpt.z.clone(), na::zero())))
            }
            else if id == 2 {
                RayIntersection::new_with_uvs(t, n, Some(Vec3::new(lpt.z.clone(), lpt.x.clone(), na::zero())))
            }
            else {
                RayIntersection::new_with_uvs(t, n, Some(Vec3::new(lpt.x.clone(), lpt.y.clone(), na::zero())))
            }
        })
    }
}

fn ray_aabb(aabb: &AABB, ray: &Ray) -> Option<(N, V, int)> {
    let mut tmax: N = Bounded::max_value();
    let mut tmin: N = -tmax;
    let mut side = 0;
    let mut diag = false;

    for i in range(0u, na::dim::<V>()) {
        if ray.dir.at(i).is_zero() {
            if ray.orig.at(i) < aabb.mins().at(i) || ray.orig.at(i) > aabb.maxs().at(i) {
                return None
            }
        }
        else {
            let _1: N = na::one();
            let denom = _1 / ray.dir.at(i);
            let flip_sides;
            let mut inter_with_near_plane = (aabb.mins().at(i) - ray.orig.at(i)) * denom;
            let mut inter_with_far_plane  = (aabb.maxs().at(i) - ray.orig.at(i)) * denom;

            if inter_with_near_plane > inter_with_far_plane {
                flip_sides = true;
                util::swap(&mut inter_with_near_plane, &mut inter_with_far_plane)
            }
            else {
                flip_sides = false;
            }

            if inter_with_near_plane > tmin {
                tmin = inter_with_near_plane;
                side = if flip_sides { -(i as int + 1) } else { i as int + 1 };
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

    if tmin < na::cast(0.0) {
        return None;
    }

    if diag {
        Some((tmin, -na::normalize(&ray.dir), side))
    }
    else {
        let mut normal: V = na::zero();

        if side < 0 {
            normal.set((-side - 1) as uint, na::one::<N>());
        }
        else {
            normal.set((side - 1) as uint, -na::one::<N>());
        }
        Some((tmin, normal, side))
    }
}
