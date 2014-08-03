use std::num::Zero;
use nalgebra::na::{Cast, FloatVec, Cross};
use nalgebra::na;
use bounding_volume;

/// Computes the circumcircle of a triangle.
pub fn circumcircle<N: Float + Cast<f64>, V: FloatVec<N> + Clone>(pa: &V, pb: &V, pc: &V) -> (V, N) {
    let a = *pa - *pc;
    let b = *pb - *pc;

    let na = na::sqnorm(&a);
    let nb = na::sqnorm(&b);

    let dab = na::dot(&a, &b);

    let _2: N = na::cast(2.0);
    let denom = _2 * (na * nb - dab * dab);

    if denom.is_zero() {
        // the triangle is degenerate
        // FIXME: do something smarter?
        bounding_volume::point_cloud_bounding_sphere(&[pa.clone(), pb.clone(), pc.clone()])
    }
    else {
        let k = b * na - a * nb;

        let center = (a * na::dot(&k, &b) - b * na::dot(&k, &a)) / denom + *pc;
        let radius = na::norm(&(*pa - center));

        (center, radius)
    }
}

/// Tests if three points are exactly aligned.
pub fn is_affinely_dependent_triangle<V: Sub<V, V> + Cross<AV>, AV: Zero>(p1: &V, p2: &V, p3: &V) -> bool
{
    let p1p2 = *p2 - *p1;
    let p1p3 = *p3 - *p1;

    na::cross(&p1p2, &p1p3).is_zero()
}

/// Tests if a point is inside of a triangle.
pub fn is_point_in_triangle<N: Float, V: FloatVec<N>>(p: &V, p1: &V, p2: &V, p3: &V) -> bool {
    let p1p2 = *p2 - *p1;
    let p2p3 = *p3 - *p2;
    let p3p1 = *p1 - *p3;

    let p1p = *p - *p1;
    let p2p = *p - *p2;
    let p3p = *p - *p3;

    let d11 = na::dot(&p1p, &p1p2);
    let d12 = na::dot(&p2p, &p2p3);
    let d13 = na::dot(&p3p, &p3p1);

    d11 >= na::zero() && d11 <= na::sqnorm(&p1p2) &&
    d12 >= na::zero() && d12 <= na::sqnorm(&p2p3) &&
    d13 >= na::zero() && d13 <= na::sqnorm(&p3p1)
}
