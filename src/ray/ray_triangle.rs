use nalgebra::na::Identity;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use geom::Triangle;
use ray::{Ray, RayCast, RayCastWithTransform, gjk_toi_and_normal_with_ray};
use math::{N, V};

#[cfg(dim3)]
use nalgebra::na;
#[cfg(dim3)]
use std::num::Zero;

impl RayCast for Triangle {
    #[cfg(dim3)]
    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<(N, V)> {
        if self.margin().is_zero() {
            triangle_ray_intersection(self.a(), self.b(), self.c(), ray).map(|(toi, n, _)| (toi, n))
        }
        else {
            gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<V>::new_w_tls(), ray)
        }
    }

    #[cfg(dim2)]
    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<(N, V)> {
        // FIXME: optimize that!
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<V>::new_w_tls(), ray)
    }

    #[cfg(dim4)]
    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<(N, V)> {
        gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<V>::new_w_tls(), ray)
    }
}

impl RayCastWithTransform for Triangle { }

/// Computes the intersection between a triangle and a ray.
///
/// If an intersection is found, the time of impact, the normal and the barycentric coordinates of
/// the intersection point are returned.
#[cfg(dim3)]
pub fn triangle_ray_intersection(a: &V, b: &V, c: &V, ray: &Ray) -> Option<(N, V, (N, N, N))> {
    let ab = *b - *a;
    let ac = *c - *a;

    // normal
    let n = na::cross(&ab, &ac);
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
    let e = -na::cross(&ray.dir, &ap);

    let mut v;
    let mut w;
    let toi;
    let normal;

    if t < na::zero() {
        v = -na::dot(&ac, &e);

        if (v < na::zero() || v > d) {
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

        if (v < na::zero() || v > d) {
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

    Some((toi, normal, (-v - w + na::one(), v, w)))
}
