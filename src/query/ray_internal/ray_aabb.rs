use num::Bounded;
use std::mem;

#[cfg(feature = "dim3")]
use na::Point2;
use na::{self, Real};

use bounding_volume::AABB;
use math::{Isometry, Vector};
use query::{Ray, RayCast, RayIntersection};

impl<N: Real> RayCast<N> for AABB<N> {
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, solid: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut tmin: N = na::zero();
        let mut tmax: N = Bounded::max_value();

        for i in 0usize..na::dimension::<Vector<N>>() {
            if ls_ray.dir[i].is_zero() {
                if ls_ray.origin[i] < self.mins()[i] || ls_ray.origin[i] > self.maxs()[i] {
                    return None;
                }
            } else {
                let _1: N = na::one();
                let denom = _1 / ls_ray.dir[i];
                let mut inter_with_near_plane = (self.mins()[i] - ls_ray.origin[i]) * denom;
                let mut inter_with_far_plane = (self.maxs()[i] - ls_ray.origin[i]) * denom;

                if inter_with_near_plane > inter_with_far_plane {
                    mem::swap(&mut inter_with_near_plane, &mut inter_with_far_plane)
                }

                tmin = tmin.max(inter_with_near_plane);
                tmax = tmax.min(inter_with_far_plane);

                if tmin > tmax {
                    return None;
                }
            }
        }

        if tmin.is_zero() && !solid {
            Some(tmax)
        } else {
            Some(tmin)
        }
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        ray_aabb(self, &ls_ray, solid).map(|(t, n, _)| RayIntersection::new(t, m * n))
    }

    #[cfg(feature = "dim3")]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        do_toi_and_normal_and_uv_with_ray(m, self, ray, solid)
    }
}

#[cfg(feature = "dim3")]
fn do_toi_and_normal_and_uv_with_ray<N: Real>(
    m: &Isometry<N>,
    aabb: &AABB<N>,
    ray: &Ray<N>,
    solid: bool,
) -> Option<RayIntersection<N>> {
    if na::dimension::<Vector<N>>() != 3 {
        aabb.toi_and_normal_with_ray(m, ray, solid)
    } else {
        let ls_ray = ray.inverse_transform_by(m);

        ray_aabb(aabb, &ls_ray, solid).map(|(t, n, s)| {
            let pt = ls_ray.origin + ls_ray.dir * t;
            let dpt = pt - *aabb.mins();
            let scale = *aabb.maxs() - *aabb.mins();
            let id = na::abs(&s);
            let gs_n = m * n;

            if id == 1 {
                RayIntersection::new_with_uvs(
                    t,
                    gs_n,
                    Some(Point2::new(dpt[1] / scale[1], dpt[2] / scale[2])),
                )
            } else if id == 2 {
                RayIntersection::new_with_uvs(
                    t,
                    gs_n,
                    Some(Point2::new(dpt[2] / scale[2], dpt[0] / scale[0])),
                )
            } else {
                RayIntersection::new_with_uvs(
                    t,
                    gs_n,
                    Some(Point2::new(dpt[0] / scale[0], dpt[1] / scale[1])),
                )
            }
        })
    }
}

fn ray_aabb<N: Real>(aabb: &AABB<N>, ray: &Ray<N>, solid: bool) -> Option<(N, Vector<N>, isize)> {
    let mut tmax: N = Bounded::max_value();
    let mut tmin: N = -tmax;
    let mut near_side = 0;
    let mut far_side = 0;
    let mut near_diag = false;
    let mut far_diag = false;

    for i in 0usize..na::dimension::<Vector<N>>() {
        if ray.dir[i].is_zero() {
            if ray.origin[i] < aabb.mins()[i] || ray.origin[i] > aabb.maxs()[i] {
                return None;
            }
        } else {
            let _1: N = na::one();
            let denom = _1 / ray.dir[i];
            let flip_sides;
            let mut inter_with_near_plane = (aabb.mins()[i] - ray.origin[i]) * denom;
            let mut inter_with_far_plane = (aabb.maxs()[i] - ray.origin[i]) * denom;

            if inter_with_near_plane > inter_with_far_plane {
                flip_sides = true;
                mem::swap(&mut inter_with_near_plane, &mut inter_with_far_plane)
            } else {
                flip_sides = false;
            }

            if inter_with_near_plane > tmin {
                tmin = inter_with_near_plane;
                near_side = if flip_sides {
                    -(i as isize + 1)
                } else {
                    i as isize + 1
                };
                near_diag = false;
            } else if inter_with_near_plane == tmin {
                near_diag = true;
            }

            if inter_with_far_plane < tmax {
                tmax = inter_with_far_plane;
                far_side = if !flip_sides {
                    -(i as isize + 1)
                } else {
                    i as isize + 1
                };
                far_diag = false;
            } else if inter_with_far_plane == tmax {
                far_diag = true;
            }

            if tmin > tmax {
                return None;
            }
        }
    }

    if tmin < na::convert(0.0f64) {
        // the ray starts inside of the box
        if solid {
            Some((na::zero(), na::zero(), far_side))
        } else {
            if far_diag {
                Some((tmax, -na::normalize(&ray.dir), far_side))
            } else {
                let mut normal = Vector::zeros();

                if far_side < 0 {
                    normal[(-far_side - 1) as usize] = -N::one();
                } else {
                    normal[(far_side - 1) as usize] = N::one();
                }

                Some((tmax, normal, far_side))
            }
        }
    } else {
        if near_diag {
            Some((tmin, -na::normalize(&ray.dir), near_side))
        } else {
            let mut normal = Vector::zeros();

            if near_side < 0 {
                normal[(-near_side - 1) as usize] = N::one();
            } else {
                normal[(near_side - 1) as usize] = -N::one();
            }
            Some((tmin, normal, near_side))
        }
    }
}
