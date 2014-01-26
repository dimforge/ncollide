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
    fn toi_with_ray(&self, ray: &Ray, solid: bool) -> Option<N> {
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

        if tmin.is_zero() && !solid {
            Some(tmax)
        }
        else {
            Some(tmin)
        }
    }

    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        ray_aabb(self, ray, solid).map(|(t, n, _)| RayIntersection::new(t, n))
    }

    #[cfg(dim3)]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        ray_aabb(self, ray, solid).map(|(t, n, s)| {
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

fn ray_aabb(aabb: &AABB, ray: &Ray, solid: bool) -> Option<(N, V, int)> {
    let mut tmax: N   = Bounded::max_value();
    let mut tmin: N   = -tmax;
    let mut near_side = 0;
    let mut far_side  = 0;
    let mut near_diag = false;
    let mut far_diag  = false;

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
                tmin      = inter_with_near_plane;
                near_side = if flip_sides { -(i as int + 1) } else { i as int + 1 };
                near_diag = false;
            }
            else if inter_with_near_plane == tmin {
                near_diag = true;
            }

            if inter_with_far_plane < tmax {
                tmax     = inter_with_far_plane;
                far_side = if !flip_sides { -(i as int + 1) } else { i as int + 1 };
                far_diag = false;
            }
            else if inter_with_far_plane == tmax {
                far_diag = true;
            }

            if tmin > tmax {
                return None;
            }
        }
    }

    if tmin < na::cast(0.0) {
        // the ray starts inside of the box
        if solid {
            Some((na::zero(), na::zero(), far_side))
        }
        else {
            if far_diag {
                Some((tmax, -na::normalize(&ray.dir), far_side))
            }
            else {
                let mut normal: V = na::zero();

                if far_side < 0 {
                    normal.set((-far_side - 1) as uint, -na::one::<N>());
                }
                else {
                    normal.set((far_side - 1) as uint, na::one::<N>());
                }

                Some((tmax, normal, far_side))
            }
        }
    }
    else {
        if near_diag {
            Some((tmin, -na::normalize(&ray.dir), near_side))
        }
        else {
            let mut normal: V = na::zero();

            if near_side < 0 {
                normal.set((-near_side - 1) as uint, na::one::<N>());
            }
            else {
                normal.set((near_side - 1) as uint, -na::one::<N>());
            }
            Some((tmin, normal, near_side))
        }
    }
}
