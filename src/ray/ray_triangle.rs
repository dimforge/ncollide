use std::num::Zero;
use na::{Vec3, Identity, Transform, Rotate};
use na;
use narrow_phase::algorithm::johnson_simplex::JohnsonSimplex;
use shape::Triangle;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection, implicit_toi_and_normal_with_ray};
use math::{Scalar, Point, Vect};

use utils;

impl<N, P, V> LocalRayCast<N, P, V> for Triangle<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        if na::dim::<P>() == 3 {
            triangle_ray_intersection(self.a(), self.b(), self.c(), ray).map(|(r, _)| r)
        }
        else {
            implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, P, V>::new_w_tls(), ray, solid)
        }
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for Triangle<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

/// Computes the intersection between a triangle and a ray.
///
/// If an intersection is found, the time of impact, the normal and the barycentric coordinates of
/// the intersection point are returned.
pub fn triangle_ray_intersection<N, P, V>(a: &P, b: &P, c: &P, ray: &Ray<P, V>)
                                          -> Option<(RayIntersection<N, V>, Vec3<N>)>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let ab = *b - *a;
    let ac = *c - *a;

    // normal
    let n = utils::cross3(&ab, &ac);
    let d = na::dot(&n, &ray.dir);

    // the normal and the ray direction are parallel
    if d.is_zero() {
        return None;
    }

    let ap = ray.orig - *a;
    let t  = na::dot(&ap, &n);

    // the ray does not intersect the plane defined by the triangle
    if (t < na::zero() && d < na::zero()) ||
       (t > na::zero() && d > na::zero()) {
        return None;
    }

    let d = d.abs();

    //
    // intersection: compute barycentric coordinates
    //
    let e = -utils::cross3(&ray.dir, &ap);

    let mut v;
    let mut w;
    let toi;
    let normal;

    if t < na::zero() {
        v = -na::dot(&ac, &e);

        if v < na::zero() || v > d {
            return None;
        }

        w = na::dot(&ab, &e);

        if w < na::zero() || v + w > d {
            return None;
        }

        let invd = na::one::<N>() / d;
        toi      = -t * invd;
        normal   = -na::normalize(&n);
        v        = v * invd;
        w        = w * invd;
    }
    else {
        v = na::dot(&ac, &e);

        if v < na::zero() || v > d {
            return None;
        }

        w = -na::dot(&ab, &e);

        if w < na::zero() || v + w > d {
            return None;
        }

        let invd = na::one::<N>() / d;
        toi      = t * invd;
        normal   = na::normalize(&n);
        v        = v * invd;
        w        = w * invd;
    }

    Some((RayIntersection::new(toi, normal), Vec3::new(-v - w + na::one(), v, w)))
}
