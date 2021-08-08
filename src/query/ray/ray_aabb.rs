use crate::num::Bounded;
use std::mem;

#[cfg(feature = "dim3")]
use na::Point2;
use na::{self, RealField};

use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector, DIM};
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::{FeatureId, Segment};

impl<N: RealField + Copy> RayCast<N> for AABB<N> {
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, max_toi: N, solid: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut tmin: N = na::zero();
        let mut tmax: N = max_toi;

        for i in 0usize..DIM {
            if ls_ray.dir[i].is_zero() {
                if ls_ray.origin[i] < self.mins[i] || ls_ray.origin[i] > self.maxs[i] {
                    return None;
                }
            } else {
                let _1: N = na::one();
                let denom = _1 / ls_ray.dir[i];
                let mut inter_with_near_plane = (self.mins[i] - ls_ray.origin[i]) * denom;
                let mut inter_with_far_plane = (self.maxs[i] - ls_ray.origin[i]) * denom;

                if inter_with_near_plane > inter_with_far_plane {
                    mem::swap(&mut inter_with_near_plane, &mut inter_with_far_plane)
                }

                tmin = tmin.max(inter_with_near_plane);
                tmax = tmax.min(inter_with_far_plane);

                if tmin > tmax {
                    // This covers the case where tmax is negative because tmin is
                    // initialized at zero.
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
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        ray_aabb(self, &ls_ray, max_toi, solid).map(|(t, n, i)| {
            let feature = if i < 0 {
                FeatureId::Face(-i as usize - 1 + 3)
            } else {
                FeatureId::Face(i as usize - 1)
            };

            RayIntersection::new(t, m * n, feature)
        })
    }

    #[cfg(feature = "dim3")]
    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        do_toi_and_normal_and_uv_with_ray(m, self, ray, max_toi, solid)
    }
}

impl<N: RealField + Copy> AABB<N> {
    /// Computes the parameters of the two intersection points between a line and this AABB.
    ///
    /// The parameters are such that the point are given by `orig + dir * parameter`.
    /// Returns `None` if there is no intersection.
    #[inline]
    pub fn clip_line_parameters(&self, orig: &Point<N>, dir: &Vector<N>) -> Option<(N, N)> {
        clip_line(self, orig, dir).map(|clip| ((clip.0).0, (clip.1).0))
    }

    /// Computes the intersection segment between a line and this AABB.
    ///
    /// Returns `None` if there is no intersection.
    #[inline]
    pub fn clip_line(&self, orig: &Point<N>, dir: &Vector<N>) -> Option<Segment<N>> {
        clip_line(self, orig, dir)
            .map(|clip| Segment::new(orig + dir * (clip.0).0, orig + dir * (clip.1).0))
    }

    /// Computes the parameters of the two intersection points between a ray and this AABB.
    ///
    /// The parameters are such that the point are given by `ray.orig + ray.dir * parameter`.
    /// Returns `None` if there is no intersection.
    #[inline]
    pub fn clip_ray_parameters(&self, ray: &Ray<N>) -> Option<(N, N)> {
        self.clip_line_parameters(&ray.origin, &ray.dir)
            .and_then(|clip| {
                let t0 = clip.0;
                let t1 = clip.1;

                if t1 < N::zero() {
                    None
                } else {
                    Some((t0.max(N::zero()), t1))
                }
            })
    }

    /// Computes the intersection segment between a ray and this AABB.
    ///
    /// Returns `None` if there is no intersection.
    #[inline]
    pub fn clip_ray(&self, ray: &Ray<N>) -> Option<Segment<N>> {
        self.clip_ray_parameters(ray)
            .map(|clip| Segment::new(ray.point_at(clip.0), ray.point_at(clip.1)))
    }
}

#[cfg(feature = "dim3")]
fn do_toi_and_normal_and_uv_with_ray<N: RealField + Copy>(
    m: &Isometry<N>,
    aabb: &AABB<N>,
    ray: &Ray<N>,
    max_toi: N,
    solid: bool,
) -> Option<RayIntersection<N>> {
    if DIM != 3 {
        aabb.toi_and_normal_with_ray(m, ray, max_toi, solid)
    } else {
        let ls_ray = ray.inverse_transform_by(m);

        ray_aabb(aabb, &ls_ray, max_toi, solid).map(|(t, n, s)| {
            let pt = ls_ray.origin + ls_ray.dir * t;
            let dpt = pt - aabb.mins;
            let scale = aabb.maxs - aabb.mins;
            let id = s.abs();
            let gs_n = m * n;
            let feature = if s < 0 {
                FeatureId::Face(id as usize - 1 + 3)
            } else {
                FeatureId::Face(id as usize - 1)
            };

            if id == 1 {
                RayIntersection::new_with_uvs(
                    t,
                    gs_n,
                    feature,
                    Some(Point2::new(dpt[1] / scale[1], dpt[2] / scale[2])),
                )
            } else if id == 2 {
                RayIntersection::new_with_uvs(
                    t,
                    gs_n,
                    feature,
                    Some(Point2::new(dpt[2] / scale[2], dpt[0] / scale[0])),
                )
            } else {
                RayIntersection::new_with_uvs(
                    t,
                    gs_n,
                    feature,
                    Some(Point2::new(dpt[0] / scale[0], dpt[1] / scale[1])),
                )
            }
        })
    }
}

fn clip_line<N: RealField + Copy>(
    aabb: &AABB<N>,
    origin: &Point<N>,
    dir: &Vector<N>,
) -> Option<((N, Vector<N>, isize), (N, Vector<N>, isize))> {
    // NOTE: we don't start with tmin = 0 so we can return the correct normal
    // when the ray starts exactly on the object contour.

    let mut tmax: N = Bounded::max_value();
    let mut tmin: N = -tmax;
    let mut near_side = 0;
    let mut far_side = 0;
    let mut near_diag = false;
    let mut far_diag = false;

    for i in 0usize..DIM {
        if dir[i].is_zero() {
            if origin[i] < aabb.mins[i] || origin[i] > aabb.maxs[i] {
                return None;
            }
        } else {
            let _1: N = na::one();
            let denom = _1 / dir[i];
            let flip_sides;
            let mut inter_with_near_plane = (aabb.mins[i] - origin[i]) * denom;
            let mut inter_with_far_plane = (aabb.maxs[i] - origin[i]) * denom;

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

            if tmax < N::zero() || tmin > tmax {
                return None;
            }
        }
    }

    let near = if near_diag {
        (tmin, -dir.normalize(), near_side)
    } else {
        let mut normal = Vector::zeros();

        if near_side < 0 {
            normal[(-near_side - 1) as usize] = N::one();
        } else {
            normal[(near_side - 1) as usize] = -N::one();
        }

        (tmin, normal, near_side)
    };

    let far = if far_diag {
        (tmax, -dir.normalize(), far_side)
    } else {
        let mut normal = Vector::zeros();

        if far_side < 0 {
            normal[(-far_side - 1) as usize] = -N::one();
        } else {
            normal[(far_side - 1) as usize] = N::one();
        }

        (tmax, normal, far_side)
    };

    Some((near, far))
}

fn ray_aabb<N: RealField + Copy>(
    aabb: &AABB<N>,
    ray: &Ray<N>,
    max_toi: N,
    solid: bool,
) -> Option<(N, Vector<N>, isize)> {
    clip_line(aabb, &ray.origin, &ray.dir).and_then(|(near, far)| {
        if near.0 < N::zero() {
            if solid {
                Some((na::zero(), na::zero(), far.2))
            } else if far.0 <= max_toi {
                Some(far)
            } else {
                None
            }
        } else if near.0 <= max_toi {
            Some(near)
        } else {
            None
        }
    })
}
