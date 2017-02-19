use num::Zero;

use alga::general::{Real, Id};
use na::{self, Vector3};

use query::algorithms::johnson_simplex::JohnsonSimplex;
use query::{Ray, RayCast, RayIntersection};
use query::ray_internal;
use shape::Triangle;
use math::{Point, Isometry};

use utils;

impl<P: Point, M: Isometry<P>> RayCast<P, M> for Triangle<P> {
    #[inline]
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vector>> {
        let ls_ray = ray.inverse_transform_by(m);

        let res = if na::dimension::<P::Vector>() == 3 {
            triangle_ray_intersection(self.a(), self.b(), self.c(), &ls_ray).map(|(r, _)| r)
        }
        else {
            ray_internal::implicit_toi_and_normal_with_ray(&Id::new(), self,
                                                           &mut JohnsonSimplex::<P>::new_w_tls(),
                                                           &ls_ray,
                                                           solid)
        };

        res.map(|mut r| { r.normal = m.rotate_vector(&r.normal); r })
    }
}

/// Computes the intersection between a triangle and a ray.
///
/// If an intersection is found, the time of impact, the normal and the barycentric coordinates of
/// the intersection point are returned.
pub fn triangle_ray_intersection<P: Point>(a: &P, b: &P, c: &P, ray: &Ray<P>)
                                    -> Option<(RayIntersection<P::Vector>, Vector3<P::Real>)> {
    let ab = *b - *a;
    let ac = *c - *a;

    // normal
    let n = utils::cross3(&ab, &ac);
    let d = na::dot(&n, &ray.dir);

    // the normal and the ray direction are parallel
    if d.is_zero() {
        return None;
    }

    let ap = ray.origin - *a;
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

        let invd = na::one::<P::Real>() / d;
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

        let invd = na::one::<P::Real>() / d;
        toi      = t * invd;
        normal   = na::normalize(&n);
        v        = v * invd;
        w        = w * invd;
    }

    Some((RayIntersection::new(toi, normal), Vector3::new(-v - w + na::one(), v, w)))
}
