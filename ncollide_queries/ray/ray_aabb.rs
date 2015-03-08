use std::mem;
use na::{Transform, Rotate, Pnt2, Bounded};
use na;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use entities::bounding_volume::AABB;
use math::{Scalar, Point, Vect};


impl<N, P, V> LocalRayCast<N, P, V> for AABB<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn toi_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        let mut tmin: N = na::zero();
        let mut tmax: N = Bounded::max_value();

        for i in 0usize .. na::dim::<P>() {
            if na::is_zero(&ray.dir[i]) {
                if ray.orig[i] < (*self.mins())[i] || ray.orig[i] > (*self.maxs())[i] {
                    return None
                }
            }
            else {
                let _1: N = na::one();
                let denom = _1 / ray.dir[i];
                let mut inter_with_near_plane = ((*self.mins())[i] - ray.orig[i]) * denom;
                let mut inter_with_far_plane  = ((*self.maxs())[i] - ray.orig[i]) * denom;

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

        if na::is_zero(&tmin) && !solid {
            Some(tmax)
        }
        else {
            Some(tmin)
        }
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        ray_aabb(self, ray, solid).map(|(t, n, _)| RayIntersection::new(t, n))
    }

    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        do_toi_and_normal_and_uv_with_ray(self, ray, solid)
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for AABB<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

fn do_toi_and_normal_and_uv_with_ray<N, P, V>(aabb: &AABB<P>, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    if na::dim::<P>() != 3 {
        aabb.toi_and_normal_with_ray(ray, solid)
    }
    else {
        ray_aabb(aabb, ray, solid).map(|(t, n, s)| {
            let pt    = ray.orig + ray.dir * t;
            let dpt   = pt - *aabb.mins();
            let scale = *aabb.maxs() - *aabb.mins();
            let id    = na::abs(&s);

            if id == 1 {
                RayIntersection::new_with_uvs(t, n, Some(Pnt2::new(dpt[1] / scale[1], dpt[2] / scale[2])))
            }
            else if id == 2 {
                RayIntersection::new_with_uvs(t, n, Some(Pnt2::new(dpt[2] / scale[2], dpt[0] / scale[0])))
            }
            else {
                RayIntersection::new_with_uvs(t, n, Some(Pnt2::new(dpt[0] / scale[0], dpt[1] / scale[1])))
            }
        })
    }
}

fn ray_aabb<N, P, V>(aabb: &AABB<P>, ray: &Ray<P, V>, solid: bool) -> Option<(N, V, isize)>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let mut tmax: N   = Bounded::max_value();
    let mut tmin: N   = -tmax;
    let mut near_side = 0;
    let mut far_side  = 0;
    let mut near_diag = false;
    let mut far_diag  = false;

    for i in 0usize .. na::dim::<P>() {
        if na::is_zero(&ray.dir[i]) {
            if ray.orig[i] < (*aabb.mins())[i] || ray.orig[i] > (*aabb.maxs())[i] {
                return None
            }
        }
        else {
            let _1: N = na::one();
            let denom = _1 / ray.dir[i];
            let flip_sides;
            let mut inter_with_near_plane = ((*aabb.mins())[i] - ray.orig[i]) * denom;
            let mut inter_with_far_plane  = ((*aabb.maxs())[i] - ray.orig[i]) * denom;

            if inter_with_near_plane > inter_with_far_plane {
                flip_sides = true;
                mem::swap(&mut inter_with_near_plane, &mut inter_with_far_plane)
            }
            else {
                flip_sides = false;
            }

            if inter_with_near_plane > tmin {
                tmin      = inter_with_near_plane;
                near_side = if flip_sides { -(i as isize + 1) } else { i as isize + 1 };
                near_diag = false;
            }
            else if inter_with_near_plane == tmin {
                near_diag = true;
            }

            if inter_with_far_plane < tmax {
                tmax     = inter_with_far_plane;
                far_side = if !flip_sides { -(i as isize + 1) } else { i as isize + 1 };
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

    if tmin < na::cast(0.0f64) {
        // the ray starts inside of the box
        if solid {
            Some((na::zero(), na::zero(), far_side))
        }
        else {
            if far_diag {
                Some((tmax, -na::normalize(&ray.dir), far_side))
            }
            else {
                let mut normal = na::zero::<V>();

                if far_side < 0 {
                    normal[(-far_side - 1) as usize] = -na::one::<N>();
                }
                else {
                    normal[(far_side - 1) as usize] = na::one::<N>();
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
            let mut normal = na::zero::<V>();

            if near_side < 0 {
                normal[(-near_side - 1) as usize] = na::one::<N>();
            }
            else {
                normal[(near_side - 1) as usize] = -na::one::<N>();
            }
            Some((tmin, normal, near_side))
        }
    }
}
