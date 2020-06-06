use na::{self, RealField, Vector3};

use crate::math::{Isometry, Point};
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::{FeatureId, Triangle};

impl<N: RealField> RayCast<N> for Triangle<N> {
    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        _: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);
        let mut inter = ray_intersection_with_triangle(&self.a, &self.b, &self.c, &ls_ray)?.0;

        if inter.toi <= max_toi {
            inter.normal = m * inter.normal;
            Some(inter)
        } else {
            None
        }
    }
}

/// Computes the intersection between a triangle and a ray.
///
/// If an intersection is found, the time of impact, the normal and the barycentric coordinates of
/// the intersection point are returned.
pub fn ray_intersection_with_triangle<N: RealField>(
    a: &Point<N>,
    b: &Point<N>,
    c: &Point<N>,
    ray: &Ray<N>,
) -> Option<(RayIntersection<N>, Vector3<N>)> {
    let ab = *b - *a;
    let ac = *c - *a;

    // normal
    let n = ab.cross(&ac);
    let d = n.dot(&ray.dir);

    // the normal and the ray direction are parallel
    if d.is_zero() {
        return None;
    }

    let ap = ray.origin - *a;
    let t = ap.dot(&n);

    // the ray does not intersect the plane defined by the triangle
    if (t < na::zero() && d < na::zero()) || (t > na::zero() && d > na::zero()) {
        return None;
    }

    let fid = if d < N::zero() { 0 } else { 1 };

    let d = d.abs();

    //
    // intersection: compute barycentric coordinates
    //
    let e = -ray.dir.cross(&ap);

    let mut v;
    let mut w;
    let toi;
    let normal;

    if t < na::zero() {
        v = -ac.dot(&e);

        if v < na::zero() || v > d {
            return None;
        }

        w = ab.dot(&e);

        if w < na::zero() || v + w > d {
            return None;
        }

        let invd = na::one::<N>() / d;
        toi = -t * invd;
        normal = -n.normalize();
        v = v * invd;
        w = w * invd;
    } else {
        v = ac.dot(&e);

        if v < na::zero() || v > d {
            return None;
        }

        w = -ab.dot(&e);

        if w < na::zero() || v + w > d {
            return None;
        }

        let invd = na::one::<N>() / d;
        toi = t * invd;
        normal = n.normalize();
        v = v * invd;
        w = w * invd;
    }

    Some((
        RayIntersection::new(toi, normal, FeatureId::Face(fid)),
        Vector3::new(-v - w + na::one(), v, w),
    ))
}
