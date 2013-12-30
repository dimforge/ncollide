use std::num::Zero;
use nalgebra::na::{AlgebraicVecExt, Cast, Identity, Rotate, Transform, VecExt};
use nalgebra::na;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use geom::Triangle;
use ray::{Ray, RayCast, RayCastWithTransform, gjk_toi_and_normal_with_ray};


impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Triangle<N, V> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        if na::dim::<V>() == 3 && self.margin().is_zero() {
            triangle_ray_intersection(self.a(), self.b(), self.c(), ray).map(|(toi, n, _)| (toi, n))
        }
        else {
            gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
        }
    }
}

impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Triangle<N, V> { }

pub fn triangle_ray_intersection<N: Num + Ord + Signed + Algebraic,
                                 V: AlgebraicVecExt<N>>(
                                 a:   &V,
                                 b:   &V,
                                 c:   &V,
                                 ray: &Ray<V>)
                                 -> Option<(N, V, (N, N, N))> {
    assert!(na::dim::<V>() == 3);

    /*
     * NOTE: redefine the cross product in order to not require the `Cross` trait.
     */
    #[inline(always)]
    fn cross<N: Num, V: VecExt<N>>(a: &V, b: &V) -> V {
        let mut res = na::zero::<V>();

        res.set(0, a.at(1) * b.at(2) - a.at(2) * b.at(1));
        res.set(1, a.at(2) * b.at(0) - a.at(0) * b.at(2));
        res.set(2, a.at(0) * b.at(1) - a.at(1) * b.at(0));

        res
    }

    let ab = *b - *a;
    let ac = *c - *a;

    // normal
    let n = cross(&ab, &ac);
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
    let e = -cross(&ray.dir, &ap);

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
