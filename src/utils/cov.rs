use std::fmt::Show;
use std::num::Zero;
use nalgebra::na::{Cast, Outer, FloatVec, RMul, Inv};
use nalgebra::na;
use utils;

// FIXME: move this to nalgebra?

/// Computes the convariance matrix of a set of points.
pub fn cov<N: Cast<f64>, V: FloatVec<N> + Outer<M>, M: Mul<N, M> + Add<M, M> + Zero>(pts: &[V]) -> M {
    cov_and_center(pts).val0()
}

/// Computes the covariance matrix and center of a set of points.
pub fn cov_and_center<N: Cast<f64>,
                      V: FloatVec<N> + Outer<M>,
                      M: Mul<N, M> + Add<M, M> + Zero>(
                      pts: &[V])
                      -> (M, V) {
    let center        = utils::center(pts);
    let mut cov: M    = na::zero();
    let normalizer: N = na::cast(1.0 / (pts.len() as f64));

    for p in pts.iter() {
        let cp = *p - center;
        cov = cov + na::outer(&cp, &cp) * normalizer;
    }

    (cov, center)
}

/// Centers and reduces a set of data.
///
/// Returns the covariance matrix, the center of the data, and a boolean that is `true` if the
/// operation succeeded (otherwise, the returned value as valid, by the input points are left
/// unchanged).
pub fn center_reduce<N: Cast<f64>,
                     V: FloatVec<N> + Outer<M>,
                     M: Mul<N, M> + Add<M, M> + Zero + RMul<V> + Inv + Show>(
                     pts: &mut [V])
                     -> (M, V, bool) {
    let (cov, center) = cov_and_center(pts);

    match na::inv(&cov) {
        None => (cov, center, false),
        Some(icov) => {
            for pt in pts.mut_iter() {
                *pt = icov.rmul(&(*pt - center));
            }

            (cov, center, true)
        }
    }
}
