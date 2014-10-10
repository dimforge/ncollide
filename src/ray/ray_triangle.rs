use na::{Vec3, Identity};
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use geom::Triangle;
use ray::{Ray, RayCast, RayIntersection, implicit_toi_and_normal_with_ray};
use math::{Scalar, Point, Vect};

#[cfg(feature = "3d")]
use na;
#[cfg(feature = "3d")]
use std::num::Zero;

#[cfg(feature = "3d")]
impl RayCast for Triangle {
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<Point, Vect>::new_w_tls(), ray, solid)
    }
}

#[cfg(feature = "2d")]
impl RayCast for Triangle {
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        // FIXME: optimize that!
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<Point, Vect>::new_w_tls(), ray, solid)
    }
}

#[cfg(feature = "4d")]
impl RayCast for Triangle {
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        implicit_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<Point, Vect>::new_w_tls(), ray, solid)
    }
}

/// Not yet implemented in dimensions other than 3.
#[cfg(not(feature = "3d"))]
pub fn triangle_ray_intersection(_: &Point, _: &Point, _: &Point, _: &Ray) -> Option<(RayIntersection, Vec3<Scalar>)> {
    fail!("`triangle_ray_intersection` is not yet implemented for dimensions other than 3.")
}

/// Computes the intersection between a triangle and a ray.
///
/// If an intersection is found, the time of impact, the normal and the barycentric coordinates of
/// the intersection point are returned.
#[cfg(feature = "3d")]
pub fn triangle_ray_intersection(a: &Point, b: &Point, c: &Point, ray: &Ray) -> Option<(RayIntersection, Vec3<Scalar>)> {
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

        if v < na::zero() || v > d {
            return None;
        }

        w = na::dot(&ab, &e);

        if w < na::zero() || v + w > d {
            return None;
        }

        let invd = na::one::<Scalar>() / d;
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

        let invd = na::one::<Scalar>() / d;
        toi      = t * invd;
        normal   = na::normalize(&n);
        v        = v * invd;
        w        = w * invd;
    }

    Some((RayIntersection::new(toi, normal), Vec3::new(-v - w + na::one(), v, w)))
}
