use nalgebra::na::{Cast, FloatVec};
use nalgebra::na;

/// Computes the circumcircle of a triangle.
pub fn circumcircle<N: Float + Cast<f64>, V: FloatVec<N>>(pa: &V, pb: &V, pc: &V) -> (V, N) {
    let a = *pa - *pc;
    let b = *pb - *pc;

    let na = na::sqnorm(&a);
    let nb = na::sqnorm(&b);

    let dab = na::dot(&a, &b);

    let _2: N = na::cast(2.0);
    let denom = _2 * (na * nb - dab * dab);
    let k     = b * na - a * nb;

    let center = (a * na::dot(&k, &b) - b * na::dot(&k, &a)) / denom + *pc;
    let radius = na::norm(&(*pa - center));

    (center, radius)
}
